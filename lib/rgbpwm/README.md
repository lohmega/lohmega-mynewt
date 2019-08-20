# RGBLeds Application


```
nrfjprog -f NRF52 -e;nrfjprog -f NRF52 --reset --program rgbpwm_fullmem.hex
```

Open uart with 115200-8n1. Commands avaiable include:  

## Change local colour

```
rgb set 0xWWRRGGBB [delay ms]

ex:
rgb set 0x220022 1000
rgb set 0xaaaaaa 2000
```

## Change remote colour(s)

- **address** is a 16-bit address expressed in hex. 0xffff - all nodes

```
rgb tx <address> 0xWWRRGGBB [delay ms]

ex:
rgb tx 0xffff 0x220022 1000
rgb tx 0x0123 0xaaaaaa 2000
```

# Configuration

The app stores settings in flash to retain them between power cycles.
The available settings can be shown with the ```config dump``` command:

```
051598 compat> config dump
052598 panmstr/pan_id = 0xDECA
052598 uwb/channel = 5
...
052601 uwb/role = 0x0000                                 # This board's role in the network
052601 split/status = 0
052601 rgbpwm/pwm_freq = 1000                            # Frequency of the LED-PWM
052601 rgbpwm/ldelay = 35000                             # Delay between automatic local colour changes
052602 rgbpwm/colours0 = #ff0000,#00ff00,#0000ff,#000000 # Set of approved colours
052603 rgbpwm/colours1 =                                 # approved colours continued...
052603 rgbpwm/colours2 =                                 # approved colours continued...
052603 rgbpwm/colours3 =                                 # approved colours continued...
...
052606 reboot/written = 

```

To change a configuration parameter use the following command flow:

```
config <parameter to change> <new value>
config commit                 # Activate changed value
config save                   # Save in permanent memory
```

## Setting the role

Only one board can be the Master of the network, the others are slaves.
By default a board is a slave that connects to a local master. A slave
has the uwb/role parameter set to 0x0 whilst the master has it set to 0x7.

To promote a board to a master:

```
config uwb/role 0x7
config save
reset
```

The last command is to restart the board as it's only at boot it will
set itself up for the correct role. 

## Example: To the pwm frequency on the local board:

```
config rgbpwm/pwm_freq 800    # Change the frequency of the pwm to 800 hz
config commit                 # Activate change
config save                   # Save in permanent memory so it's retained after power cycle
```

## Changing the list of approved colours the board shifts between

If not controlled over the network the board automatically chooses a new colour
to change to every ```rgbpwm/ldelay``` milliseconds. It chooses the new colour
from the list of approved colours stored in the ```rgbpwm/colours0``` to ```rgbpwm/colours3```
vectors. The reason there are more than one vector is that only 64 characters can be stored
in each configuration parameter.

Each colour is represented as a html-hex code with some caviates:

- Hexadecimal starting with White, Red, Green, and Blue: ```#WWRRGGBB```
- Each colour starts with ```#```
- Each colour can range from ```00```(0) to ```FF```(255) where ```FF``` is the brightest.
- The white channel may or may not be used on the board itself
- The short ```#RGB``` format is not allowed.
- Not all colours need to be included. For instance ```#000000FF``` (blue) is the same as ```#FF```.
- Lower or upper case allowed: ```#aaBBcc == #AAbbCC```

To store a set of new colours, a max of 32 colours allowed:

```
config rgbpwm/colours0 #010101,#020202,#050505,#FF00FF,#EECC99,#123456,#987654,#ABCDEF
config rgbpwm/colours1 #110101,#120202,#050505,#eF00FF,#3ECC99,#323456,#f87654,#fBCDEF
config commit   # Activate
config save     # Save in permanent memory
config dump     # Show current settings
```

## Listing the active nodes in the network

```
panm list
```
