from typing import Optional, Any, Dict, List, Iterable, Tuple, Literal, Callable
from functools import partial
import warnings
from qcodes.utils.helpers import create_on_off_val_mapping
from qcodes.instrument import IPInstrument, InstrumentChannel, InstrumentModule
from qcodes.parameters import Parameter
from time import sleep


class OrthogonalFlagsMap:
    """
    This is a mapping helper for various flags-to-strings facilities
    (Harp warnings, Harp flags, ...) encoded as an i.e. a bitwise field
    of *several* items. You can iterate through the warnings to reach
    all of them:
       # >>> warn = OrthogonalFlagsMap(HarpWarnings, 0x441)
       # >>> [ warn.text(w) for w in warn ]
       # [ 'Sync rate is zero', 'Input rate is too high', 'Time span is too small' ]
       # >>> warn.INPT_RATE_ZERO
       # True
       # >>> warn.INPT_RATE_TOO_HIGH
       # False
    """

    def __init__(self, flags_map: Dict[str, Tuple[int, str]], code: int = None) -> None:
        """
        Initializes the mapper. Parameters:
          - `code`: This is a bitwise field of flags that this instance represents.
          - `flags_map`: This is a dictionary which maps single string keys to
            `(flagMask, flagDescription)` tuples. The string key part
            is a non-changeable string that describes the flags for all eternity,
            and which the user (or other program layers) can use to access the flag.
            `flagDescription` is a human-readable string which can change, e.g.
            by translation or a more precise specification, and `flagMask` is a bit
            mask that indicates whether the flag is set or not.
        """
        self.flagsMap: Dict[str, Tuple[int, str]] = flags_map
        self.code: int = code

        if code is not None:
            self.recode(code)

    def recode(self, code: int) -> 'OrthogonalFlagsMap':
        """
        Resets the code to `code` and returns a reference to self.
        This is to update the active/inactive flags list to the
        ones encoded in `code` without altering the identity of the
        object.
        """
        self.code = code
        return self

    def __str__(self) -> str:
        return str([f for f in self.keys()])

    def __getattr__(self, key: str) -> bool:
        return (self.code & self.flagsMap[key][0]) != 0

    def __iter__(self) -> Iterable[str]:
        """ Iterate through all warnings encoded in `self.code`. """
        for k, v in self.flagsMap.items():
            if (v[0] & self.code) != 0:
                yield k

    def keys(self) -> Iterable[str]:
        """
        Mimic a bit of a `dict`-like interface: return all the HHLIB API
        warning keys that are encoded in `self.code`.
        """
        for k in self:
            yield k

    def items(self) -> Iterable[Tuple[str, str]]:
        """
        Mimic a bit more a `dict`-like interface: return all the HHLIB API
        warning keys that are encoded in `self.code`.
        """
        for k, v in self.flagsMap.items():
            if v[0] & self.code:
                yield k, v[1]

    def __getitem__(self, flag: str) -> bool:
        """ Another way of reading a flag """
        return self.__getattr__(flag)

    def text(self, flag: str) -> str:
        """ Returns the description text. """
        return self.flagsMap.get(flag, (None, None))[1]

    def mask(self, flag: str) -> int:
        """ Returns the numerical mask value. """
        return self.flagsMap.get(flag, (None, None))[0]

    def __len__(self) -> int:
        return len([i for i in self.items()])


AxisStatusFlags = {
    'READY': (0x0001, "Axis ready"),
    'REF_OK': (0x0002, "Reference position installed"),
    'LIMIT_NEG': (0x0004, "Negative limit switch active"),
    'LIMIT_POS': (0x0008, "Positive limit switch active"),
    'HOMED': (0x0010, "Axis homed / initialized"),
    'ENABLED': (0x0020, "Axis enabled"),
    'BUSY': (0x0040, "Execution in progress"),
    'IDLE': (0x0080, "Controller idle / all axes stopped"),
    'OSCILLATING': (0x0100, "Oscillation in progress"),
    'OSC_ERROR': (0x0200, "Oscillation error"),
    'ENC_REF_OK': (0x0400, "Encoder reference installed"),
    'LIMIT_SNEG': (0x1000, "Negative soft limit"),
    'LIMIT_SPOS': (0x2000, "Positive soft limit"),
    'BLOCKED': (0x4000, "Controller blocked"),
    'ERROR': (0x8000, "Error message pending"),
    'EXT_STOP': (0x10000, "External stop active")
}


class HuberSmc(IPInstrument):
    """
    Class to represent a Huber SMC 9300 motor controller.

    Args:
        name (str): name for the instrument
        address (str): IP address for the resource to connect
        port (int): Port to connect IP
        timeout (float, optional). Visa timeout. Defaults to 20s.
    """

    def __init__(
            self,
            name: str,
            address: str,
            port: int,
            actuator_names: Optional[List[str]] = None,
            timeout: Optional[float] = 20,
            write_terminator: str = '\r\n',
            read_terminator: str = '\r\n',
            wait_for_movement: bool = False,
            **kwargs: Any) -> None:
        super().__init__(
            name,
            address=address,
            port=port,
            terminator=write_terminator,
            timeout=timeout,
            write_confirmation=False,
            **kwargs)

        self._address = address
        self._port = port
        self._read_terminator = read_terminator
        self._wait_for_movement = wait_for_movement
        self._actuator_names = actuator_names
        self.timeout = timeout

        self.flush_connection()

        self._create_parameters()
        self._create_channels(actuator_names)

        self.connect_message()
        self.update()

    def flush_connection(self) -> None:
        try:
            super().flush_connection()
        except TimeoutError:
            pass

    @property
    def actuator_names(self) -> List[str]:
        if self._actuator_names:
            return self._actuator_names
        else:
            names = [ax.name for ax in self.axes]
            return names

    def _create_channels(self, actuator_names) -> None:
        self._channels = []

        for channel_number in range(1, self.n_axes() + 1):
            axis_config = self.ask_raw(f'?conf{channel_number}').split('\r\n')[1:]
            axis_config = {x.split(':')[0][:-1]: x.split(':')[1] for x in axis_config}

            if actuator_names is not None:
                actuator_name = actuator_names[channel_number - 1]
            else:
                actuator_name = None

            channel = HuberAxis(
                parent=self,
                num=channel_number,
                name=actuator_name,
                config=axis_config
            )

            self.add_submodule(f"channel{channel_number}", channel)
            self._channels.append(channel)

    def _create_parameters(self) -> None:
        self.add_parameter(
            'n_axes',
            docstring='Number of installed axes on the controller.',
            get_cmd=lambda: self.num_axes
        )

        # self.add_parameter(
        #     'front_panel_locked',
        #     docstring='Stop the device reacting to the hardware controls.',
        #     get_cmd=lambda: self._get_thorlabs_attribute(getter_name='GetFrontPanelLocked'),
        #     set_cmd=lambda x: self._set_thorlabs_attribute(x, getter_name='SetFrontPanelLock'),
        #     val_mapping=create_on_off_val_mapping(
        #         on_val=True, off_val=False)
        #     )

        self.add_parameter(
            'wait_for_movement',
            get_cmd=lambda: self._wait_for_movement,
            set_cmd=lambda x: setattr(self, '_wait_for_movement', x),
            val_mapping=create_on_off_val_mapping(on_val=True, off_val=False),
            docstring='Wait for the move operation to finish.'
        )

        self.add_parameter(
            'model',
            get_cmd=lambda: self.ask_raw('?v').split('\r\n')[0].split(' ')[0],
            docstring='Serial model of the device, read-only.'
        )

        self.add_parameter(
            'firmware_version',
            get_cmd=lambda: self.ask_raw('?v').split('\r\n')[0].split(' ')[1],
            docstring='Serial model of the device, read-only.'
        )

    def _setup_controller(self) -> None:
        conf_cmds = [
            'pqrf0',  # set position query format to NOT include unit
        ]
        for cmd in conf_cmds:
            self.send_raw(cmd)

    @staticmethod
    def _get_ax_from_msg(msg) -> int:
        result = ''
        for char in msg:
            if char in '1234567890':
                result += char
        return int(result)

    def get_idn(self) -> Dict[str, str]:
        """
        Overwrites the get_idn method to provide ID information.

        Returns:
            A dictionary containing keys 'vendor', 'model', 'serial', and
            'firmware', and their corresponding values.
        """
        vendor = 'Huber'
        model = self.model()
        firmware = self.firmware_version()

        idparts = [vendor, model, firmware]

        return dict(zip(("vendor", "model", "firmware"), idparts))

    def stop(self) -> None:
        self.send_raw('q')

    def ask_raw(self, cmd: str) -> str:
        # with self._huber_lock:
        with self._ensure_connection:
            self._send(cmd)
            return self._recv()[:-len(self._read_terminator)]

    def send_raw(self, cmd: str) -> None:
        with self._ensure_connection:
            self._send(cmd)

    def ask_axval(self, ax_num, query) -> str:
        if query[0] != '?':
            msg = f'Your query {query} is not a valid huber query (needs to start with ?)!'
            warnings.warn(msg)
            return ''
        rtrn = self.ask_raw(f'{query}{ax_num}')
        rtrn_ax, val = rtrn.split(':')
        rtrn_ax = self._get_ax_from_msg(rtrn_ax)
        if rtrn_ax != ax_num:
            msg = f'Asked status for axis {ax_num}, return message for axis {rtrn_ax}!'
            warnings.warn(msg)
            return ''
        return val.strip(';')

    def update(self) -> None:
        try:
            self.flush_connection()
        except TimeoutError:
            pass

        for axes in self.axes:
            params = axes.parameters
            for key in params:
                params[key]()

            for key in axes.submodules:
                params = axes.submodules[key].parameters
                for kkey in params:
                    params[kkey]()

    @property
    def axes(self) -> List['HuberAxis']:
        return self._channels

    @property
    def num_axes(self) -> int:
        recv = self.ask_raw('?s').split(';')
        recv = [x for x in recv if len(x) > 0]
        return len(recv)


class HuberAxis(InstrumentChannel):
    def __init__(self, parent: HuberSmc, num: int, config: Dict[str, Any], name: str = None) -> None:
        if name is None:
            name = f"ax{num}"

        super().__init__(parent, name)

        self.orientation = config['alias'].split('~')[1]
        self.num: int = num
        self._config: Dict[str, Any] = config
        self.controller: HuberSmc = parent
        self._get_val = partial(parent.ask_axval, num)
        self.n_decimals: int = int(config['dcpl'])

        if config['type'] == '1':
            self.position = Parameter(
                'position',
                get_cmd=lambda: self._get_val('?p'),
                get_parser=float,
                set_cmd=self.move_abs,
                label='Position',
                unit=config['unit'],
                instrument=self
            )
        elif config['type'] == '0':
            self.position = Parameter(
                'position',
                get_cmd=lambda: self._get_val('?p'),
                get_parser=float,
                set_cmd=self.move_abs,
                label='Rotation',
                unit=config['unit'],
                instrument=self
            )

        self.direction = Parameter(
            'direction',
            get_cmd=lambda: self.orientation,
            label='Axis direction acoording to controller',
            instrument=self
        )

        # Add status and configuration submodules
        status = HuberAxisStatus(
            parent=self.controller,
            name=f'ax{num}_status',
            ax_num=num
        )

        ax_conf = HuberAxisConfiguration(
            parent=self.controller,
            name=f'ax{num}_configuration',
            ax_num=num,
            config=config
        )

        self.add_submodule(f'ax{num}_status', status)
        self.add_submodule(f'ax{num}_configuration', ax_conf)

    def stop(self) -> None:
        self.controller.send_raw(f'quit{self.num}')

    def home(self, direction: Literal['+', '-'] = '+', speed: int = 10000) -> None:
        if direction not in ['+', '-']:
            warnings.warn("Direction must be in ['+', '-']")
            return
        self.controller.send_raw(f'home{self.num}:jg{direction}{speed}')

    def move_abs(self, val: float) -> None:
        self.controller.send_raw(f'goto{self.num}:{val}')
        self.wait_for_movement()

    def wait_for_movement(self) -> None:
        if not self.controller.wait_for_movement():
            return
        self.controller.set_timeout(2)
        for submodule in self.submodules.values():
            if type(submodule) is HuberAxisStatus:
                submodule.update_status()
                while 'IDLE' not in submodule.status_flags:
                    sleep(0.1)
                    submodule.update_status()
        self.controller.set_timeout(self.controller.timeout)

    def move_rel(self, val: float) -> None:
        self.controller.send_raw(f'move{self.num}:{val}')
        self.wait_for_movement()

    def move_step(self, direction: Literal['+', '-']) -> None:
        if direction not in ['+', '-']:
            warnings.warn("Step direction must be in ['+', '-']")
            return
        self.controller.send_raw(f'step{self.num}{direction}')

    def set_zero(self) -> None:
        self.controller.send_raw(f'zero{self.num}')

    def set_pos(self, pos: float) -> None:
        self.controller.send_raw(f'pos{self.num}:{pos:.{self.n_decimals}f}')


class HuberAxisConfiguration(InstrumentModule):
    def __init__(self, parent: HuberSmc, name: str, ax_num: int, config: dict):
        super().__init__(parent, name)

        self._send_raw = parent.send_raw
        self._get_val = partial(parent.ask_axval, ax_num)
        self.configuration = config

        self.add_parameter(
            'acc',
            initial_cache_value=config['acc'],
            get_cmd=lambda: self._get_val('?acc'),
            # set_cmd=partial(self._set_val(), 'acc'),
            docstring='acc{axis}:[value]  set acceleration',
            snapshot_get=False
        )

        self.add_parameter(
            'alias',
            initial_cache_value=config['alias'],
            get_cmd=lambda: self._get_val('?alias'),
            # set_cmd=partial(self._set_val(), 'alias'),
            docstring='alias{axis}:[text]{~[fn]}  set alias and full name for axis display',
            snapshot_get=False
        )

        self.add_parameter(
            'biss_ch',
            initial_cache_value=config['biss_ch'],
            get_cmd=lambda: self._get_val('?biss_ch'),
            # set_cmd=partial(self._set_val(), 'biss_ch'),
            docstring='biss_ch{axis}:[channel]  assign axis to BiSS.at96 encoder channel',
            snapshot_get=False
        )

        self.add_parameter(
            'biss_e',
            initial_cache_value=config['biss_e'],
            get_cmd=lambda: self._get_val('?biss_e'),
            # set_cmd=partial(self._set_val(), 'biss_e'),
            docstring='biss_e{axis}:[res]  set BiSS encoder system resolution [deg|mm]',
            snapshot_get=False
        )

        self.add_parameter(
            'biss_i',
            initial_cache_value=config['biss_i'],
            get_cmd=lambda: self._get_val('?biss_i'),
            # set_cmd=partial(self._set_val(), 'biss_i'),
            docstring='biss_i{axis}:[inc]  set BiSS encoder system increment 1|5|50[nm]',
            snapshot_get=False
        )

        self.add_parameter(
            'biss_id',
            initial_cache_value=config['biss_id'],
            get_cmd=lambda: self._get_val('?biss_id'),
            # set_cmd=partial(self._set_val(), 'biss_id'),
            docstring='biss_id{axis}:[id]  set BiSS device serial number',
            snapshot_get=False
        )

        self.add_parameter(
            'biss_if',
            initial_cache_value=config['biss_if'],
            get_cmd=lambda: self._get_val('?biss_if'),
            # set_cmd=partial(self._set_val(), 'biss_if'),
            docstring='biss_if{axis}:[model]  set BiSS adapter model: 4:MB4U, 5:MB5U',
            snapshot_get=False
        )

        self.add_parameter(
            'biss_t',
            initial_cache_value=config['biss_t'],
            get_cmd=lambda: self._get_val('?biss_t'),
            # set_cmd=partial(self._set_val(), 'biss_t'),
            docstring='biss_t{axis}:[travel]  set BiSS encoder system travel range',
            snapshot_get=False
        )

        self.add_parameter(
            'bln',
            initial_cache_value=config['bln'],
            get_cmd=lambda: self._get_val('?bln'),
            # set_cmd=partial(self._set_val(), 'bln'),
            docstring='bln{axis}:[-1...1]  backlash loop distance, neg. motion direction',
            snapshot_get=False
        )

        self.add_parameter(
            'blp',
            initial_cache_value=config['blp'],
            get_cmd=lambda: self._get_val('?blp'),
            # set_cmd=partial(self._set_val(), 'blp'),
            docstring='blp{axis}:[-1...1]  backlash loop distance, pos. motion direction, range',
            snapshot_get=False
        )

        self.add_parameter(
            'cf',
            initial_cache_value=config['cf'],
            get_cmd=lambda: self._get_val('?cf'),
            # set_cmd=partial(self._set_val(), 'cf'),
            docstring='cf{axis}[value]  define a calibration factor for linear encoder operation. if the encoder scale '
                      'shows a linear deviation resulting from distension of the encoder grid tape, this deviation can '
                      'be corrected. [0.99...1.01]',
            snapshot_get=False
        )

        self.add_parameter(
            'corr',
            initial_cache_value=config['corr'],
            get_cmd=lambda: self._get_val('?corr'),
            # set_cmd=partial(self._set_val(), 'corr'),
            docstring='corr{axis}:[0|1]  enable/disable position error correction',
            snapshot_get=False
        )

        self.add_parameter(
            'dcpl',
            initial_cache_value=config['dcpl'],
            get_cmd=lambda: self._get_val('?dcpl'),
            # set_cmd=partial(self._set_val(), 'dcpl'),
            docstring='dcpl{axis}:[1...8]  position display decimal places',
            snapshot_get=False
        )

        self.add_parameter(
            'dec',
            initial_cache_value=config['dec'],
            get_cmd=lambda: self._get_val('?dec'),
            # set_cmd=partial(self._set_val(), 'dec'),
            docstring='dec{axis}:[value]  set deceleration',
            snapshot_get=False
        )

        self.add_parameter(
            'ecl',
            initial_cache_value=config['ecl'],
            get_cmd=lambda: self._get_val('?ecl'),
            # set_cmd=partial(self._set_val(), 'ecl'),
            docstring='ecl{axis}:[0|1]  encoder closed loop operation',
            snapshot_get=False
        )

        self.add_parameter(
            'eclst',
            initial_cache_value=config['eclst'],
            get_cmd=lambda: self._get_val('?eclst'),
            # set_cmd=partial(self._set_val(), 'eclst'),
            docstring='eclst{axis}:[0|1]  set position display to target position value after successful execution of '
                      'the positioning task.',
            snapshot_get=False
        )

        self.add_parameter(
            'ecp',
            initial_cache_value=config['ecp'],
            get_cmd=lambda: self._get_val('?ecp'),
            # set_cmd=partial(self._set_val(), 'ecp'),
            docstring='ecp{axis}:[0|1|21|22]  EnDat protocol version',
            snapshot_get=False
        )

        self.add_parameter(
            'ect',
            initial_cache_value=config['ect'],
            get_cmd=lambda: self._get_val('?ect'),
            # set_cmd=partial(self._set_val(), 'ect'),
            docstring='ect{axis}:[type]  encoder counter type',
            snapshot_get=False
        )

        self.add_parameter(
            'edev',
            initial_cache_value=config['edev'],
            get_cmd=lambda: self._get_val('?edev'),
            # set_cmd=partial(self._set_val(), 'edev'),
            docstring='edev{axis}:[value]  max. setpoint deviation',
            snapshot_get=False
        )

        self.add_parameter(
            'edir',
            initial_cache_value=config['edir'],
            get_cmd=lambda: self._get_val('?edir'),
            # set_cmd=partial(self._set_val(), 'edir'),
            docstring='edir{axis}:[0|1]  encoder rotation direction: 0:normal, 1:inverted',
            snapshot_get=False
        )

        self.add_parameter(
            'ehst',
            initial_cache_value=config['ehst'],
            get_cmd=lambda: self._get_val('?ehst'),
            # set_cmd=partial(self._set_val(), 'ehst'),
            docstring='ehst{axis}:[interval]  encoder home position settling time',
            snapshot_get=False
        )

        self.add_parameter(
            'eias',
            initial_cache_value=config['eias'],
            get_cmd=lambda: self._get_val('?eias'),
            # set_cmd=partial(self._set_val(), 'eias'),
            docstring='eias{axis}:[0|1]  configuration of the encoder index signal level which is considered active '
                      'during homing procedures.',
            snapshot_get=False
        )

        self.add_parameter(
            'emips',
            initial_cache_value=config['emips'],
            get_cmd=lambda: self._get_val('?emips'),
            # set_cmd=partial(self._set_val(), 'emips'),
            docstring='emips{axis}:[-1...255]  required input port status to enable motion',
            snapshot_get=False
        )

        self.add_parameter(
            'emode',
            initial_cache_value=config['emode'],
            get_cmd=lambda: self._get_val('?emode'),
            # set_cmd=partial(self._set_val(), 'emode'),
            docstring='emode{axis}:[value]  set additional encoder operation modes',
            snapshot_get=False
        )

        self.add_parameter(
            'epc',
            initial_cache_value=config['epc'],
            get_cmd=lambda: self._get_val('?epc'),
            # set_cmd=partial(self._set_val(), 'epc'),
            docstring='epc{axis}:[1|0]  enable/disable protection of encoder counter content',
            snapshot_get=False
        )

        self.add_parameter(
            'eres',
            initial_cache_value=config['eres'],
            get_cmd=lambda: self._get_val('?eres'),
            # set_cmd=partial(self._set_val(), 'eres'),
            docstring='eres{axis}:[value]  encoder resolution',
            snapshot_get=False
        )

        self.add_parameter(
            'erofs',
            initial_cache_value=config['erofs'],
            get_cmd=lambda: self._get_val('?erofs'),
            # set_cmd=partial(self._set_val(), 'erofs'),
            docstring='erofs{axis}:[value]  set absolute encoder reference offset',
            snapshot_get=False
        )

        self.add_parameter(
            'esh',
            initial_cache_value=config['esh'],
            get_cmd=lambda: self._get_val('?esh'),
            # set_cmd=partial(self._set_val(), 'esh'),
            docstring='esh{axis}:[0|1]  display encoder position',
            snapshot_get=False
        )

        self.add_parameter(
            'esm',
            initial_cache_value=config['esm'],
            get_cmd=lambda: self._get_val('?esm'),
            # set_cmd=partial(self._set_val(), 'esm'),
            docstring='esm{axis}:[1|2|4]  encoder signal multiplicator',
            snapshot_get=False
        )

        self.add_parameter(
            'est',
            initial_cache_value=config['est'],
            get_cmd=lambda: self._get_val('?est'),
            # set_cmd=partial(self._set_val(), 'est'),
            docstring='est{axis}:[0|1|2]  encoder signal type',
            snapshot_get=False
        )

        self.add_parameter(
            'ffast',
            initial_cache_value=config['ffast'],
            get_cmd=lambda: self._get_val('?ffast'),
            # set_cmd=partial(self._set_val(), 'ffast'),
            docstring='ffast|vfast{axis}:[value]  fast speed setting',
            snapshot_get=False
        )

        self.add_parameter(
            'frun',
            initial_cache_value=config['frun'],
            get_cmd=lambda: self._get_val('?frun'),
            # set_cmd=partial(self._set_val(), 'frun'),
            docstring='frun|vrun{axis}:[value]  run speed',
            snapshot_get=False
        )

        self.add_parameter(
            'gden',
            initial_cache_value=config['gden'],
            get_cmd=lambda: self._get_val('?gden'),
            # set_cmd=partial(self._set_val(), 'gden'),
            docstring='gden|gn{axis}:[value]  gear factor denominator',
            snapshot_get=False
        )

        self.add_parameter(
            'gnum',
            initial_cache_value=config['gnum'],
            get_cmd=lambda: self._get_val('?gnum'),
            # set_cmd=partial(self._set_val(), 'gnum'),
            docstring='gnum|gz{axis}:[value]  gear factor numerator',
            snapshot_get=False
        )

        self.add_parameter(
            'hsdm',
            initial_cache_value=config['hsdm'],
            get_cmd=lambda: self._get_val('?hsdm'),
            # set_cmd=partial(self._set_val(), 'hsdm'),
            docstring='hsdm{axis}:[0|1]  set homing SD mode: 0=fixed, 1=use DEC',
            snapshot_get=False
        )

        self.add_parameter(
            'lsa',
            initial_cache_value=config['lsa'],
            get_cmd=lambda: self._get_val('?lsa'),
            # set_cmd=partial(self._set_val(), 'lsa'),
            docstring='lsa|lsat[0|1]  limit swich assignment',
            snapshot_get=False
        )

        self.add_parameter(
            'lsnf',
            initial_cache_value=config['lsnf'],
            get_cmd=lambda: self._get_val('?lsnf'),
            # set_cmd=partial(self._set_val(), 'lsnf'),
            docstring='lsnf{axis}:[0..8]  set limit switch signal input noise filter time constant',
            snapshot_get=False
        )

        self.add_parameter(
            'macc',
            initial_cache_value=config['macc'],
            get_cmd=lambda: self._get_val('?macc'),
            # set_cmd=partial(self._set_val(), 'macc'),
            docstring='macc{axis}:[value]  manual positioning acceleration',
            snapshot_get=False
        )

        self.add_parameter(
            'mdec',
            initial_cache_value=config['mdec'],
            get_cmd=lambda: self._get_val('?mdec'),
            # set_cmd=partial(self._set_val(), 'mdec'),
            docstring='mdec{axis}:[value]  manual positioning deceleration',
            snapshot_get=False
        )

        self.add_parameter(
            'mdir',
            initial_cache_value=config['mdir'],
            get_cmd=lambda: self._get_val('?mdir'),
            # set_cmd=partial(self._set_val(), 'mdir'),
            docstring='mdir|mdr[0|1]  motor rotation direction: 0:normal, 1:inverted',
            snapshot_get=False
        )

        self.add_parameter(
            'mpr',
            initial_cache_value=config['mpr'],
            get_cmd=lambda: self._get_val('?mpr'),
            # set_cmd=partial(self._set_val(), 'mpr'),
            docstring='mpr{axis}:[1...500]  max. position retries for closed loop positioning',
            snapshot_get=False
        )

        self.add_parameter(
            'mres',
            initial_cache_value=config['mres'],
            get_cmd=lambda: self._get_val('?mres'),
            # set_cmd=partial(self._set_val(), 'mres'),
            docstring='mres{axis}:[resolution]  set motor step resolution',
            snapshot_get=False
        )

        self.add_parameter(
            'ofs',
            initial_cache_value=config['ofs'],
            get_cmd=lambda: self._get_val('?ofs'),
            # set_cmd=partial(self._set_val(), 'ofs'),
            docstring='ofs[1...16777214]  set distance for SetFindSteps() (only temporary setting).',
            snapshot_get=False
        )

        self.add_parameter(
            'prst',
            initial_cache_value=config['prst'],
            get_cmd=lambda: self._get_val('?prst'),
            # set_cmd=partial(self._set_val(), 'prst'),
            docstring='prst{axis}:[value]  position retry settling time',
            snapshot_get=False
        )

        self.add_parameter(
            'prt',
            initial_cache_value=config['prt'],
            get_cmd=lambda: self._get_val('?prt'),
            # set_cmd=partial(self._set_val(), 'prt'),
            docstring='prt{axis}:[value]  position retry threshold',
            snapshot_get=False
        )

        self.add_parameter(
            'rfs',
            initial_cache_value=config['rfs'],
            get_cmd=lambda: self._get_val('?rfs'),
            # set_cmd=partial(self._set_val(), 'rfs'),
            docstring='rfs[1...16777214]  set run-free steps (only temporary setting).',
            snapshot_get=False
        )

        self.add_parameter(
            'rofs',
            initial_cache_value=config['rofs'],
            get_cmd=lambda: self._get_val('?rofs'),
            # set_cmd=partial(self._set_val(), 'rofs'),
            docstring='rofs|nofs{axis}:[value]  set reference offset',
            snapshot_get=False
        )

        self.add_parameter(
            'sdm',
            initial_cache_value=config['sdm'],
            get_cmd=lambda: self._get_val('?sdm'),
            # set_cmd=partial(self._set_val(), 'sdm'),
            docstring='sdm{axis}:[0|1]  set SD mode: 0=SD not present, 1=SD present',
            snapshot_get=False
        )

        self.add_parameter(
            'sdpw',
            initial_cache_value=config['sdpw'],
            get_cmd=lambda: self._get_val('?sdpw'),
            # set_cmd=partial(self._set_val(), 'sdpw'),
            docstring='sdpw{axis}[interval]  set slowdown pulse width',
            snapshot_get=False
        )

        self.add_parameter(
            'slen',
            initial_cache_value=config['slen'],
            get_cmd=lambda: self._get_val('?slen'),
            # set_cmd=partial(self._set_val(), 'slen'),
            docstring='slen[0|1]  enable soft limits',
            snapshot_get=False
        )

        self.add_parameter(
            'slneg',
            initial_cache_value=config['slneg'],
            get_cmd=lambda: self._get_val('?slneg'),
            # set_cmd=partial(self._set_val(), 'slneg'),
            docstring='slneg|slmin{axis}:[pos]  set min. soft limit',
            snapshot_get=False
        )

        self.add_parameter(
            'slpos',
            initial_cache_value=config['slpos'],
            get_cmd=lambda: self._get_val('?slpos'),
            # set_cmd=partial(self._set_val(), 'slpos'),
            docstring='slpos|slmax{axis}:[pos]  set max. soft limit',
            snapshot_get=False
        )

        self.add_parameter(
            'twen',
            initial_cache_value=config['twen'],
            get_cmd=lambda: self._get_val('?twen'),
            # set_cmd=partial(self._set_val(), 'twen'),
            docstring='twen{axis}:[axis2].[fn]  define twin encoder',
            snapshot_get=False
        )

        self.add_parameter(
            'type',
            initial_cache_value=config['type'],
            get_cmd=lambda: self._get_val('?type'),
            # set_cmd=partial(self._set_val(), 'type'),
            docstring='type{axis}[0|1|2|3|5]  set type of axis 0:circle, 1: linear, 2:slit screen, 3:circle segment '
                      'type1, 5:circle segment type2',
            snapshot_get=False
        )

        self.add_parameter(
            'unit',
            initial_cache_value=config['unit'],
            get_cmd=lambda: self._get_val('?unit'),
            # set_cmd=partial(self._set_val(), 'unit'),
            docstring='unit[mm|deg]  set position display unit',
            snapshot_get=False
        )


class HuberAxisStatus(InstrumentModule):
    """
    Flags:
        attribute: docstring, flag
        limit_hw_pos: positive limit switch active., 'LIMIT_POS'
        limit_hw_neg: negative limit switch active., 'LIMIT_NEG'
        limit_hw: any limit switch active., --
        enabled: axis is enabled., 'ENABLED'
        error: error message pending., 'ERROR'
        homed: axis is homed., 'HOMED'
        pos_ref: installed position reference., 'REF_OK'
        enc_ref: installed encoder reference., 'ENC_REF_OK'
        ready: axis is ready., 'READY'
        blocked: controller is blocked., 'BLOCKED'
        idle: controller is idle (all axes are stopped)., 'IDLE'
        ext_stop: External stop active., 'EXT_STOP'
        oscillating: Oscillation in progress., 'OSCILLATING'
        osc_error: Oscillation error, 'OSC_ERROR'
        busy: Execution in progress., 'BUSY'
        limit_sw_pos: Positive soft limit., 'LIMIT_SPOS'
        limit_sw_neg: Negative soft limit., 'LIMIT_SNEG'
        limit_sw: Any software switch active., --
    """

    def __init__(
            self,
            parent: HuberSmc,
            name: str,
            ax_num: int,
    ) -> None:
        super().__init__(parent, name)
        self.ax_num: int = ax_num
        self._get_sbits: Callable[[], int] = lambda: int(parent.ask_axval(self.ax_num, '?s'))
        self._sbits: int = self._get_sbits()
        self.statusmap: OrthogonalFlagsMap = OrthogonalFlagsMap(AxisStatusFlags)
        self.statusmap.recode(self._sbits)

        self.add_parameter(
            'statusbits',
            get_cmd=lambda: self._get_sbits(),
            docstring='Statusbits of the given axis.'
        )

        self.add_parameter(
            'limit_hw_pos',
            get_cmd=lambda: self._get_flag('LIMIT_POS'),
            docstring='positive limit switch active. 0x0008'
        )
        self.add_parameter(
            'limit_hw_neg',
            get_cmd=lambda: self._get_flag('LIMIT_NEG'),
            docstring='negative limit switch active. 0x0004'
        )
        self.add_parameter(
            'limit_hw',
            get_cmd=lambda: self._any_hw_switch(),
            docstring=' any limit switch active.'
        )
        self.add_parameter(
            'enabled',
            get_cmd=lambda: self._get_flag('ENABLED'),
            docstring=' axis is enabled. 0x0020'
        )
        self.add_parameter(
            'error',
            get_cmd=lambda: self._get_flag('ERROR'),
            docstring=' error message pending. 0x8000'
        )
        self.add_parameter(
            'homed',
            get_cmd=lambda: self._get_flag('HOMED'),
            docstring=' axis is homed. 0x0010'
        )
        self.add_parameter(
            'pos_ref',
            get_cmd=lambda: self._get_flag('REF_OK'),
            docstring=' REF_OK'
        )
        self.add_parameter(
            'enc_ref',
            get_cmd=lambda: self._get_flag('ENC_REF_OK'),
            docstring=' ENC_REF_OK'
        )
        self.add_parameter(
            'ready',
            get_cmd=lambda: self._get_flag('READY'),
            docstring=' axis is ready.'
        )
        self.add_parameter(
            'blocked',
            get_cmd=lambda: self._get_flag('BLOCKED'),
            docstring=' controller is blocked.'
        )
        self.add_parameter(
            'idle',
            get_cmd=lambda: self._get_flag('IDLE'),
            docstring=' controller is idle (all axes are stopped)'
        )
        self.add_parameter(
            'ext_stop',
            get_cmd=lambda: self._get_flag('EXT_STOP'),
            docstring=' External stop active'
        )
        self.add_parameter(
            'oscillating',
            get_cmd=lambda: self._get_flag('OSCILLATING'),
            docstring=' Oscillation in progress'
        )
        self.add_parameter(
            'osc_error',
            get_cmd=lambda: self._get_flag('OSC_ERROR'),
            docstring=' Oscillation error'
        )
        self.add_parameter(
            'busy',
            get_cmd=lambda: self._get_flag('BUSY'),
            docstring=' Execution in progress.'
        )
        self.add_parameter(
            'limit_sw_pos',
            get_cmd=lambda: self._get_flag('LIMIT_SPOS'),
            docstring=' Positive soft limit.'
        )
        self.add_parameter(
            'limit_sw_neg',
            get_cmd=lambda: self._get_flag('LIMIT_SNEG'),
            docstring=' Negative soft limit.'
        )
        self.add_parameter(
            'limit_sw',
            get_cmd=lambda: self._any_sw_switch(),
            docstring='Any software switch active.'
        )

    def _any_hw_switch(self) -> bool:
        return any([
            'LIMIT_POS' in self.status_flags,
            'LIMIT_NEG' in self.status_flags
        ])

    def _any_sw_switch(self) -> bool:
        return any([
            'LIMIT_SPOS' in self.status_flags,
            'LIMIT_SNEG' in self.status_flags
        ])

    def _get_flag(self, flag) -> bool:
        self.update_status()
        if flag not in AxisStatusFlags.keys():
            warnings.warn('Provided flag {flag} is not a valid status flag')
            return False
        condition = flag in self.status_flags
        return condition

    def _update_sbits(self) -> None:
        self._sbits = self._get_sbits()

    def update_status(self) -> None:
        self._update_sbits()
        self.statusmap.recode(self._sbits)

    @property
    def status_flags(self) -> List[str]:
        self.statusmap.recode(self._sbits)
        return list(self.statusmap.__iter__())
