import logging
from time import sleep
from typing import Optional, Any, Dict, List, Literal
from functools import partial
import warnings

from qcodes.instrument import IPInstrument, InstrumentChannel, InstrumentModule
from qcodes.parameters import Parameter
from qcodes import validators as vals

log = logging.getLogger(__name__)


class OrthogonalFlagsMap:
    '''
    This is a mapping helper for various flags-to-strings facilities
    (Harp warnings, Harp flags, ...) encoded as a i.e. a bitwise field
    of *several* items. You can iterate through the warnings to reach
    all of them:
    ```
       >>> warn = OrthogonalFlagsMap(HarpWarnings, 0x441)
       >>> [ warn.text(w) for w in warn ]
       [ 'Sync rate is zero', 'Input rate is too high', 'Time span is too small' ]
       >>> warn.INPT_RATE_ZERO
       True
       >>> warn.INPT_RATE_TOO_HIGH
       False
    ```
    '''
    
    def __init__(self, flagsMap, code=None):
        '''
        Initializes the mapper. Parameters:
          - `code`: This is a bitwise field of flags that this instance represents.
          - `flagsMap`: This is a dictionary which maps single string keys to
            `(flagMask, flagDescription)` tuples. The string key part
            is a non-changeable string that describes the flags for all eternity,
            and which the user (or other program layers) can use to access the flag.
            `flagDescription` is a human-readable string which can change, e.g.
            by translation or a more precise specification, and `flagMask` is a bit
            mask that indicates whether the flag is set or not.
        '''
        self.flagsMap = flagsMap
        if code is not None:
            self.recode(code)
    
    def recode(self, code):
        '''
        Resets the code to `code` and returns a reference to self.
        This is to update the active/inactive flags list to the
        ones encoded in `code` without altering the identity of the
        object.
        '''
        self.code = code
        return self

    def __str__(self):
        return str([f for f in self.keys()])

    def __getattr__(self, key):
        return (self.code & self.flagsMap[key][0]) != 0

    def __iter__(self):
        ''' Iterate through all warnings encoded in `self.code`. '''
        for k,v in self.flagsMap.items():
            if (v[0] & self.code) != 0:
                yield k

    def keys(self):
        '''
        Mimic a bit of a `dict`-like interface: return all the HHLIB API
        warning keys that are encoded in `self.code`.
        '''
        for k in self:
            yield k

    def items(self):
        '''
        Mimic a bit more a `dict`-like interface: return all the HHLIB API
        warning keys that are encoded in `self.code`.
        '''
        for k,v in self.flagsMap.items():
            if (v[0] & self.code):
                yield (k, v[1])
    
    def __getitem__(self, flag):
        ''' Another way of reading a flag '''
        return self.__getattr__(flag)

    def text(self, flag):
        '''
        Returns the description text.
        '''
        return self.flagsMap.get(flag, None)[1]

    def mask(self, flag):
        '''
        Returns the numerical mask value.
        '''
        return self.flagsMap.get(flag, None)[0]

    def __len__(self):
        return len([i for i in self.items()])


AxisStatusFlags = {
    'READY':       (0x0001, "Axis ready"),
    'REF_OK':      (0x0002, "Reference position installed"),
    'LIMIT_NEG':   (0x0004, "Negative limit switch active"),
    'LIMIT_POS':   (0x0008, "Positive limit switch active"),
    'HOMED':       (0x0010, "Axis homed / initialized"),
    'ENABLED':     (0x0020, "Axis enabled"),
    'BUSY':        (0x0040, "Execution in progress"),
    'IDLE':        (0x0080, "Controller idle / all axes stopped"),
    'OSCILLATING': (0x0100, "Oscillation in progress"),
    'OSC_ERROR':   (0x0200, "Oscillation error"),
    'ENC_REF_OK':  (0x0400, "Encoder reference installed"),  
    'LIMIT_SNEG':  (0x1000, "Negative soft limit"),
    'LIMIT_SPOS':  (0x2000, "Positive soft limit"),
    'BLOCKED':     (0x4000, "Controller blocked"),
    'ERROR':       (0x8000, "Error message pending"),
    'EXT_STOP':   (0x10000, "External stop active") }


class Huber_SMC(IPInstrument):
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
            timeout: float = 20,
            write_terminator: str = '\r\n',
            read_terminator: str = '\r\n',
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
         
        _ = self.ask_raw('?s')
        _ = self.ask_raw('?s')
        _ = self.ask_raw('?s')
  
        self._create_parameters()
        self._create_channels(actuator_names)      

        self.connect_message()

        self.update()

    def _create_channels(self, actuator_names) -> None:
        self._channels = []

        for channel_number in range(1, self.n_axes()+1):
            axis_config = self.ask_raw(f'?conf{channel_number}').split('\r\n')[1:]
            axis_config = {x.split(':')[0][:-1]:x.split(':')[1] for x in axis_config}
        
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

    def ask_raw(self, cmd:str) -> str:
        with self._ensure_connection:
            self._send(cmd)
            return self._recv()[:-len(self._read_terminator)]
    
    def send_raw(self, cmd:str) -> None:
        with self._ensure_connection:
            self._send(cmd)

    def ask_axval(self, ax_num, query) -> str:
        if query[0] != '?':
            msg = f'Your query {query} is not a valid huber query (needs to start with ?)!'
            warnings.warn(msg)
            return
        rtrn = self.ask_raw(f'{query}{ax_num}')
        rtrn_ax, val = rtrn.split(':')
        rtrn_ax = self._get_ax_from_msg(rtrn_ax)
        if rtrn_ax != ax_num:
            msg = f'Asked status for axis {ax_num}, return message for axis {rtrn_ax}!'
            warnings.warn(msg)
            return
        return val.strip(';')

    def update(self) -> None:
        for axes in self.axes:
            params = axes.parameters
            for key in params:
                params[key]()

            for key in axes.submodules:
                params = axes.submodules[key].parameters
                for key in params:
                    params[key]()
    
    @property
    def axes(self) -> list:
        return self._channels

    @property
    def num_axes(self) -> int:
        recv = self.ask_raw('?s').split(';')
        recv = [x for x in recv if len(x) > 0]
        return len(recv)


class HuberAxis(InstrumentChannel):
    def __init__(self, parent: Huber_SMC, num: int, config: dict, name: str = None) -> None:
        if name is None:
            name = f"ax{num}"
        
        self.orientation = config['alias'].split('~')[1]
        super().__init__(parent, name)

        self.num = num
        self.config = config
        self.controller = parent
        self._get_val = partial(parent.ask_axval, num)
        
        if config['type'] == '1':
            self.position = Parameter(
                'position',
                get_cmd = lambda: self._get_val('?p'),
                get_parser = float,
                set_cmd = self.move_abs,
                label='Position',
                unit=config['unit'],
                instrument=self
            )
        elif config['type'] == '0':
            self.position = Parameter(
                'position',
                get_cmd = lambda: self._get_val('?p'),
                get_parser = float,
                set_cmd = self.move_abs,
                label='Rotation',
                unit=config['unit'],
                instrument=self  
            )

        self.direction = Parameter(
            'direction',
            get_cmd = lambda: self.orientation,
            label='Axis direction acoording to controller',
            instrument=self
        )

        status = HuberAxisStatus(parent=self.controller, name=f'ax{num}_status', ax_num=num)
        self.add_submodule(f'ax{num}_status', status)
        ax_conf = HuberAxisConfiguration(parent=self.controller, name=f'ax{num}_configuration', ax_num=num)
        self.add_submodule(f'ax{num}_configuration', ax_conf)

    def home(self, direction: Literal['+', '-'] = '+', speed: int = 10000) -> None:
        if direction not in ['+', '-']:
            warnings.warn("Direction must be in ['+', '-']")
            return
        self.controller.send_raw(f'home{self.num}:jg{direction}{speed}')


    def move_abs(self, val: float) -> None:
        self.controller.send_raw(f'goto{self.num}:{val}')

    def move_rel(self, val: float) -> None:
        self.controller.send_raw(f'move{self.num}:{val}')

    def move_step(self, direction: Literal['+', '-']) -> None:
        if direction not in ['+', '-']:
            warnings.warn("Step direction must be in ['+', '-']")
            return
        self.controller.send_raw(f'step{self.num}{direction}')


class HuberAxisConfiguration(InstrumentModule):
    def __init__(self, parent: Huber_SMC, name: str, ax_num: int):
        super().__init__(parent, name)

        self._send_raw = parent.send_raw
        self._get_val = partial(parent.ask_axval, ax_num)

        self.add_parameter(
                'alias',
                get_cmd=lambda: self._get_val('?alias'),
                # set_cmd=partial(self._set_val(), 'alias'),
                docstring='placeholder'
            )
        self.add_parameter(
                'type',
                get_cmd=lambda: self._get_val('?type'),
                # set_cmd=partial(self._set_val(), 'type'),
                docstring='placeholder'
            )
        self.add_parameter(
                'unit',
                get_cmd=lambda: self._get_val('?unit'),
                # set_cmd=partial(self._set_val(), 'unit'),
                docstring='placeholder'
            )
        self.add_parameter(
                'gnum',
                get_cmd=lambda: self._get_val('?gnum'),
                # set_cmd=partial(self._set_val(), 'gnum'),
                docstring='placeholder'
            )
        self.add_parameter(
                'gden',
                get_cmd=lambda: self._get_val('?gden'),
                # set_cmd=partial(self._set_val(), 'gden'),
                docstring='placeholder'
            )
        self.add_parameter(
                'mres',
                get_cmd=lambda: self._get_val('?mres'),
                # set_cmd=partial(self._set_val(), 'mres'),
                docstring='placeholder'
            )
        self.add_parameter(
                'dcpl',
                get_cmd=lambda: self._get_val('?dcpl'),
                # set_cmd=partial(self._set_val(), 'dcpl'),
                docstring='placeholder'
            )
        self.add_parameter(
                'rofs',
                get_cmd=lambda: self._get_val('?rofs'),
                # set_cmd=partial(self._set_val(), 'rofs'),
                docstring='placeholder'
            )
        self.add_parameter(
                'blp',
                get_cmd=lambda: self._get_val('?blp'),
                # set_cmd=partial(self._set_val(), 'blp'),
                docstring='placeholder'
            )
        self.add_parameter(
                'bln',
                get_cmd=lambda: self._get_val('?bln'),
                # set_cmd=partial(self._set_val(), 'bln'),
                docstring='placeholder'
            )
        self.add_parameter(
                'mdir',
                get_cmd=lambda: self._get_val('?mdir'),
                # set_cmd=partial(self._set_val(), 'mdir'),
                docstring='placeholder'
            )
        self.add_parameter(
                'lsa',
                get_cmd=lambda: self._get_val('?lsa'),
                # set_cmd=partial(self._set_val(), 'lsa'),
                docstring='placeholder'
            )
        self.add_parameter(
                'lsnf',
                get_cmd=lambda: self._get_val('?lsnf'),
                # set_cmd=partial(self._set_val(), 'lsnf'),
                docstring='placeholder'
            )
        self.add_parameter(
                'ofs',
                get_cmd=lambda: self._get_val('?ofs'),
                # set_cmd=partial(self._set_val(), 'ofs'),
                docstring='placeholder'
            )
        self.add_parameter(
                'rfs',
                get_cmd=lambda: self._get_val('?rfs'),
                # set_cmd=partial(self._set_val(), 'rfs'),
                docstring='placeholder'
            )
        self.add_parameter(
                'hsdm',
                get_cmd=lambda: self._get_val('?hsdm'),
                # set_cmd=partial(self._set_val(), 'hsdm'),
                docstring='placeholder'
            )
        self.add_parameter(
                'sdm',
                get_cmd=lambda: self._get_val('?sdm'),
                # set_cmd=partial(self._set_val(), 'sdm'),
                docstring='placeholder'
            )
        self.add_parameter(
                'sdpw',
                get_cmd=lambda: self._get_val('?sdpw'),
                # set_cmd=partial(self._set_val(), 'sdpw'),
                docstring='placeholder'
            )
        self.add_parameter(
                'slen',
                get_cmd=lambda: self._get_val('?slen'),
                # set_cmd=partial(self._set_val(), 'slen'),
                docstring='placeholder'
            )
        self.add_parameter(
                'slneg',
                get_cmd=lambda: self._get_val('?slneg'),
                # set_cmd=partial(self._set_val(), 'slneg'),
                docstring='placeholder'
            )
        self.add_parameter(
                'slpos',
                get_cmd=lambda: self._get_val('?slpos'),
                # set_cmd=partial(self._set_val(), 'slpos'),
                docstring='placeholder'
            )
        self.add_parameter(
                'frun',
                get_cmd=lambda: self._get_val('?frun'),
                # set_cmd=partial(self._set_val(), 'frun'),
                docstring='placeholder'
            )
        self.add_parameter(
                'ffast',
                get_cmd=lambda: self._get_val('?ffast'),
                # set_cmd=partial(self._set_val(), 'ffast'),
                docstring='placeholder'
            )
        self.add_parameter(
                'acc',
                get_cmd=lambda: self._get_val('?acc'),
                # set_cmd=partial(self._set_val(), 'acc'),
                docstring='placeholder'
            )
        self.add_parameter(
                'dec',
                get_cmd=lambda: self._get_val('?dec'),
                # set_cmd=partial(self._set_val(), 'dec'),
                docstring='placeholder'
            )
        self.add_parameter(
                'macc',
                get_cmd=lambda: self._get_val('?macc'),
                # set_cmd=partial(self._set_val(), 'macc'),
                docstring='placeholder'
            )
        self.add_parameter(
                'mdec',
                get_cmd=lambda: self._get_val('?mdec'),
                # set_cmd=partial(self._set_val(), 'mdec'),
                docstring='placeholder'
            )
        self.add_parameter(
                'emips',
                get_cmd=lambda: self._get_val('?emips'),
                # set_cmd=partial(self._set_val(), 'emips'),
                docstring='placeholder'
            )
        self.add_parameter(
                'cf',
                get_cmd=lambda: self._get_val('?cf'),
                # set_cmd=partial(self._set_val(), 'cf'),
                docstring='placeholder'
            )
        self.add_parameter(
                'corr',
                get_cmd=lambda: self._get_val('?corr'),
                # set_cmd=partial(self._set_val(), 'corr'),
                docstring='placeholder'
            )
        self.add_parameter(
                'ecl',
                get_cmd=lambda: self._get_val('?ecl'),
                # set_cmd=partial(self._set_val(), 'ecl'),
                docstring='placeholder'
            )
        self.add_parameter(
                'eclst',
                get_cmd=lambda: self._get_val('?eclst'),
                # set_cmd=partial(self._set_val(), 'eclst'),
                docstring='placeholder'
            )
        self.add_parameter(
                'ecp',
                get_cmd=lambda: self._get_val('?ecp'),
                # set_cmd=partial(self._set_val(), 'ecp'),
                docstring='placeholder'
            )
        self.add_parameter(
                'ect',
                get_cmd=lambda: self._get_val('?ect'),
                # set_cmd=partial(self._set_val(), 'ect'),
                docstring='placeholder'
            )
        self.add_parameter(
                'edev',
                get_cmd=lambda: self._get_val('?edev'),
                # set_cmd=partial(self._set_val(), 'edev'),
                docstring='placeholder'
            )
        self.add_parameter(
                'edir',
                get_cmd=lambda: self._get_val('?edir'),
                # set_cmd=partial(self._set_val(), 'edir'),
                docstring='placeholder'
            )
        self.add_parameter(
                'ehst',
                get_cmd=lambda: self._get_val('?ehst'),
                # set_cmd=partial(self._set_val(), 'ehst'),
                docstring='placeholder'
            )
        self.add_parameter(
                'eias',
                get_cmd=lambda: self._get_val('?eias'),
                # set_cmd=partial(self._set_val(), 'eias'),
                docstring='placeholder'
            )
        self.add_parameter(
                'emode',
                get_cmd=lambda: self._get_val('?emode'),
                # set_cmd=partial(self._set_val(), 'emode'),
                docstring='placeholder'
            )
        self.add_parameter(
                'epc',
                get_cmd=lambda: self._get_val('?epc'),
                # set_cmd=partial(self._set_val(), 'epc'),
                docstring='placeholder'
            )
        self.add_parameter(
                'erofs',
                get_cmd=lambda: self._get_val('?erofs'),
                # set_cmd=partial(self._set_val(), 'erofs'),
                docstring='placeholder'
            )
        self.add_parameter(
                'eres',
                get_cmd=lambda: self._get_val('?eres'),
                # set_cmd=partial(self._set_val(), 'eres'),
                docstring='placeholder'
            )
        self.add_parameter(
                'esh',
                get_cmd=lambda: self._get_val('?esh'),
                # set_cmd=partial(self._set_val(), 'esh'),
                docstring='placeholder'
            )
        self.add_parameter(
                'esm',
                get_cmd=lambda: self._get_val('?esm'),
                # set_cmd=partial(self._set_val(), 'esm'),
                docstring='placeholder'
            )
        self.add_parameter(
                'est',
                get_cmd=lambda: self._get_val('?est'),
                # set_cmd=partial(self._set_val(), 'est'),
                docstring='placeholder'
            )
        self.add_parameter(
                'mpr',
                get_cmd=lambda: self._get_val('?mpr'),
                # set_cmd=partial(self._set_val(), 'mpr'),
                docstring='placeholder'
            )
        self.add_parameter(
                'prst',
                get_cmd=lambda: self._get_val('?prst'),
                # set_cmd=partial(self._set_val(), 'prst'),
                docstring='placeholder'
            )
        self.add_parameter(
                'prt',
                get_cmd=lambda: self._get_val('?prt'),
                # set_cmd=partial(self._set_val(), 'prt'),
                docstring='placeholder'
            )
        self.add_parameter(
                'twen',
                get_cmd=lambda: self._get_val('?twen'),
                # set_cmd=partial(self._set_val(), 'twen'),
                docstring='placeholder'
            )
        self.add_parameter(
                'biss_if',
                get_cmd=lambda: self._get_val('?biss_if'),
                # set_cmd=partial(self._set_val(), 'biss_if'),
                docstring='placeholder'
            )
        self.add_parameter(
                'biss_id',
                get_cmd=lambda: self._get_val('?biss_id'),
                # set_cmd=partial(self._set_val(), 'biss_id'),
                docstring='placeholder'
            )
        self.add_parameter(
                'biss_ch',
                get_cmd=lambda: self._get_val('?biss_ch'),
                # set_cmd=partial(self._set_val(), 'biss_ch'),
                docstring='placeholder'
            )
        self.add_parameter(
                'biss_e',
                get_cmd=lambda: self._get_val('?biss_e'),
                # set_cmd=partial(self._set_val(), 'biss_e'),
                docstring='placeholder'
            )
        self.add_parameter(
                'biss_i',
                get_cmd=lambda: self._get_val('?biss_i'),
                # set_cmd=partial(self._set_val(), 'biss_i'),
                docstring='placeholder'
            )
        self.add_parameter(
                'biss_t',
                get_cmd=lambda: self._get_val('?biss_t'),
                # set_cmd=partial(self._set_val(), 'biss_t'),
                docstring='placeholder'
            )

   
class HuberAxisStatus(InstrumentModule):
    """
    Attributes:
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
        parent: Huber_SMC, 
        name: str,
        ax_num: int,
    ):    
        super().__init__(parent, name)
        self.ax_num = ax_num
        self._get_sbits = lambda: int(parent.ask_axval(self.ax_num, '?s'))
        self._sbits = self._get_sbits()
        self.statusmap = OrthogonalFlagsMap(AxisStatusFlags)
        self.statusmap.recode(self._sbits)

        self.add_parameter(
            'statusbits',
            get_cmd = lambda: self._get_sbits(),
            docstring = 'Statusbits of the given axis.'
        )

        self.add_parameter(
                'limit_hw_pos',
                get_cmd=lambda: self._get_flag('LIMIT_POS'),
                docstring=' positive limit switch active. 0x0008'
            )
        self.add_parameter(
                'limit_hw_neg',
                get_cmd=lambda: self._get_flag('LIMIT_NEG'),
                docstring=' negative limit switch active. 0x0004'
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
       
    def _any_hw_switch(self):
        return any([
            'LIMIT_POS' in self._status_flags, 
            'LIMIT_NEG' in self._status_flags
            ])

    def _any_sw_switch(self):
        return any([
            'LIMIT_SPOS' in self._status_flags, 
            'LIMIT_SNEG' in self._status_flags
            ])

    def _get_flag(self, flag):
        self.update_status()
        if flag not in AxisStatusFlags.keys():
            warnings.warn('Provided flag {flag} is not a valid status flag')
            return
        condition = flag in self._status_flags
        return condition

    def _update_sbits(self):
        self._sbits = self._get_sbits()

    def update_status(self):
        self._update_sbits()
        self.statusmap.recode(self._sbits) 
    
    @property
    def _status_flags(self):
        self.statusmap.recode(self._sbits)
        return list(self.statusmap)
