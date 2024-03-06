# LED Strip

Rotorflight supports the use of addressable LED strips.  Addressable LED strips allow each LED in the strip to be programmed with a unique and independent color. This is far more advanced than the normal RGB strips which require that all the LEDs in the strip show the same color.

## LED Strip Profiles

The LED strip feature supports 4 LED strip profiles: RACE, BEACON, STATUS and STATUS_ALT. The selected profile can be changed from the CLI, OSD LED strip menu or from an adjustment channel, i.e. switch on your radio.  Take note that the adjustment channel from your radio overrides all other LED strip profile selection options.


### STATUS Profile

The STATUS profile is used to display all the information mentioned below, i.e. warning indications, Larson scanner etc, positional lights, ACL.

Addressable LED strips can be used to show information from the flight controller system and supports:

* Up to 32 LEDs.
* User definable blink patterns.
* AUX operated switching between LED profiles.
* Indicators showing pitch/roll stick positions.
* Heading/Orientation lights.
* Flight mode specific color schemes.
* Low battery warning.
* GPS state.
* RSSI level.
* Battery level.

### STATUS_ALT Profile

The STATUS_ALT profile works just like the STATUS_PROFILE, except for LEDs with the Fade overlay. LEDs with the Fade overlay will use their alternate color when the STATUS_ALT profile is active.

### RACE Profile

The RACE profile is used to set ALL strip LEDs to the selected color for racing, i.e. to identify quads based on LED color.  The LED color is fixed and no other information is displayed.


### BEACON Profile

The BEACON profile is used to find a lost heli, it flashes all LEDs white once per second.  Again in this profile no other information is displayed on the LEDs.


### LED Profile Selection

###### OPTION 1: Use the Configurator
1. Go to the *LED Strip* tab
2. Select the profile under *LED Strip Global Settings*

###### OPTION 2: Configure an adjustment range to change the LED strip profile from your radio
1. The LED strip profile selection is performed using an adjustment configured via the *Adjustments* tab.
    - Enable an adjustment by selecting *Mapped*.
    - Select the AUX channel to be used to change the LED strip profile under *Enable channel* and set the range to 900-2100us
    - Set the same AUX channel and range under *Value channel*
    - For the adjustment select *LED Profile Selection* and set the range to 1-4.
    - Save
2. Configure the AUX channel on your radio.  When this channel is changed the selected LED strip profile will change between RACE, BEACON, STATUS and STATUS_ALT. You should see the LED function change as you do this.


###### OPTION 3: Use the CLI to select the LED strip profile (i.e. not selecting the LED strip profile with your radio)
1. Open the CLI.
2. Type ```get ledstrip_profile``` followed by enter to display the currently selected LED strip profile.
3. Type ```set ledstrip_profile=x``` where x is the profile STATUS, STATUS_ALT, RACE or BEACON and press enter.
4. Type ```save``` followed by enter to save the selected LED strip profile.


###### OPTION 4: By using the OSD
1. Open the OSD menu by yawing left and pitching forward on your radio.
2. Using the pitch stick, move down to the LED Strip menu and roll right to enter the menu.
3. The profile and race color can be configured using the left stick to go back and the right stick to navigate up/down and to change the selected value.
4. Use the left stick to go to the top level menu and select save & reboot to complete.


###### RACE COLOR: The Race color can be configured using the CLI:
1. Open the CLI.
2. Type ```get ledstrip_race_color``` followed by enter to display the currently selected race color number.
3. Type ```set ledstrip_race_color=x``` where x is the required color.
4. Type ```save``` followed by enter to save the race color to be used.


###### BRIGHTNESS: The overall brightness can be configured using the CLI:
1. Open the CLI.
2. Type ```get ledstrip_brightness``` followed by enter to display the current brightness.
3. Type ```set ledstrip_brightness=x``` where x is the brightness in percentage between 5 and 100.
4. Type ```save``` followed by enter to save the brightness level to be used.


## Supported hardware

Up to 32 WS2811/WS2812 LEDs are currently supported.

WS2812 LEDs require an 800khz signal and precise timings and thus requires the use of a dedicated hardware timer.

Note: Not all WS2812 ICs use the same timings, some batches use different timings.

### Tested Hardware

* [Adafruit NeoPixel Jewel 7](https://www.adafruit.com/products/2226) (preliminary testing)
  * Measured current consumption in all white mode ~ 350 mA.
  * Fits well under motors on mini 250 quads.
* [Adafruit NeoPixel Stick](https://www.adafruit.com/products/1426) (works well)
  * Measured current consumption in all white mode ~ 350 mA.
* [Aliexpress SK6812 RBGWW strip](https://www.aliexpress.com/wholesale?SearchText=rgbw+sk6812) (works well)
  * Alternative [Adafruit NeoPixel Stick RGBW](https://www.adafruit.com/product/2869)


### WS2811 vs WS2812

The [WS2811](https://cdn-shop.adafruit.com/datasheets/WS2811.pdf) is a LED driver IC which is connected to an RGB LED. It accepts data in the form of 8 bits each of Red-Green-Blue.

The [WS2812](https://cdn-shop.adafruit.com/datasheets/WS2812.pdf) is integrated into the package of a 50:50 LED rather than as a separate device. It accepts data in the form of 8 bits each of Green-Red-Blue.

With the [SK6812](https://cdn-shop.adafruit.com/product-files/1138/SK6812+LED+datasheet+.pdf) also GRBW variants are supported, which have a fourth (white) channel and such provide a much cleaner white color.

It is thus possible, depending on the LED board/strip being used that either Red-Green-Blue or Green-Red-Blue encoding may be required. This may be controlled by setting the following.

```
set ledstrip_grb_rgb = RGB
```
or

```
set ledstrip_grb_rgb = GRB
```
or

```
set ledstrip_grb_rgb = GRBW
```

Then confirm the required setting by simply setting an LED to be green. If it lights up red, you have the wrong setting.
Afterwards check if the second LED also lights up red - if not, you might have 4-color SK6812 LEDs and would have to select GRBW.

###### Combining RGB and GBR(W) LEDs

If you're using both WS2811 and WS2812/SK6812 LEDs, set `ledstrip_grb_rgb` to `GRB` or `GRBW`. The colors of the WS2811 LEDs will then be incorrect. To correct this, indicate which LEDs use RGB by specifying those LEDs in `ledstrip_inverted_format` - a 32 bit bitmask. For example, if LED 1 and 4 use RGB, specify 1 + 8 = 9:

```
set ledstrip_inverted_format = 9
```

## Connections

WS2812 LED strips generally require a single data line, 5V and GND.

WS2812 LEDs on full brightness can consume quite a bit of current.  It is recommended to verify the current draw and ensure your supply can cope with the load. On a multirotor that uses multiple BEC ESC's you can try use a different BEC to the one the FC uses.  e.g. ESC1/BEC1 -> FC, ESC2/BEC2 -> LED strip.   It's also possible to power one half of the strip from one BEC and the other half from another BEC.  Just ensure that the GROUND is the same for all BEC outputs and LEDs.

LED Strip pin should be:
- on a separate timer
- with DMA enabled

If you have LEDs that are intermittent, flicker or show the wrong colors then drop the VIN to less than 4.7v, e.g. by using an inline
diode on the VIN to the LED strip. The problem occurs because of the difference in voltage between the data signal and the power
signal. The WS2811 LED's require the data signal (Din) to be between 0.3 * Vin (Max) and 0.7 * VIN (Min) to register valid logic
low/high signals.  The LED pin on the CPU will always be between 0v to ~3.3v, so the Vin should be 4.7v (3.3v / 0.7 = 4.71v).
Some LEDs are more tolerant of this than others.

The datasheet can be found here: [WS2812](http://www.adafruit.com/datasheets/WS2812.pdf)

## Configuration

The LED strip feature can be configured via the Configurator or the CLI.

### Configurator
First enable the *LED_STRIP* feature in the *Configuration* tab. The *LED Strip* tab should now become visible.

Now go to the *LED Strip* tab and configure the LEDs. First setup how the LEDs are laid out so that you can visualize it later as you configure and so the flight controller knows how many LEDs there are available.

There is a step by step guide on how to use the Configurator to [configure the Led Strip feature](http://blog.oscarliang.net/setup-rgb-led-cleanflight/) which was published early 2015 by Oscar Liang.

### CLI
Enable the `LED_STRIP` feature via the cli:

```
feature LED_STRIP
```

If you enable LED_STRIP feature and the feature is turned off again after a reboot then check your config does not conflict with other features, as above.

Configure the LEDs using the `led` command.

The `led` command takes either zero or two arguments - an zero-based LED number and a sequence which indicates pair of coordinates, direction flags and mode flags and a color.

If used with zero arguments it prints out the LED configuration which can be copied for future reference.

Each LED is configured using the following template: `x,y:direction:mode:color:blinkpattern:blinkpause:alternatecolor`

`x` and `y` are grid coordinates of a 0 based 16x16 grid, north west is 0,0, south east is 15,15
`direction` specifies the directions, since an LED can face in any direction it can have multiple directions.  Directions are:

 `N` - North
 `E` - East
 `S` - South
 `W` - West
 `U` - Up
 `D` - Down

For instance, an LED that faces South-east at a 45 degree downwards angle could be configured as `SED`.

Note: It is perfectly possible to configure an LED to have all directions `NESWUD` but probably doesn't make sense.

`mode` specifies the modes that should be applied an LED. A mode consists of a base function and zero or more overlays.

Each LED has one base function:

* `C` - `C`olor.
* `F` - `F`light mode & Orientation
* `A` - `A`rmed state.
* `R` - `R`ing thrust state.
* `G` - `G`PS state.
* `S` - R`S`SSI level.
* `L` - Battery `L`evel.

And each LED can have one ore more overlays:

* `W` - `W`warnings.
* `I` - `I`ndicator.
* `T` - `T`hrust state.
* `B` - `B`link mode.
* `O` - Lars`O`n Scanner (Cylon Effect).
* `V` - `V`TX Frequency.
* `K` - Flic`K`er.
* `D` - Fa`D`e.

`color` specifies the color number (0 based index). This color has a black border in the Configurator.

`blinkpattern` specifies a 16 bit bitmask which indicates when a LED should blink. Example: two fast blinks would be 5 (binary 101), two slow blinks 3855 (binary 111100001111). The `alternatecolor` is used for blinking.

`blinkpause` specifies whether blinking should pause after the blink pattern has been finished. A value of `0` doesn't pause blinking, a value of `3` pauses the blinking for three subsequent rounds.

`alternatecolor` specifies the color for: a. blinking; b. fading between `color` and `alternatecolor` if the LED profile is switched from STATUS to STATUS_ALT. The `alternatecolor` can be black if you want to turn a light off in in the STATUS_ALT profile. The alternate color has a red border in the Configurator.

Example:

```
led 0 0,15:SD:AWI:0:0:0:0
led 1 15,0:ND:AWI:0:0:0:0
led 2 0,0:ND:AWI:0:0:0:0
led 3 0,15:SD:AWI:0:0:0:0
led 4 7,7::C:1:0:0:0
led 5 8,8::C:2:0:0:0
led 6 8,9::CB:1:3855:0:2
```

To erase an LED, and to mark the end of the chain, use `0,0::` as the second argument, like this:

```
led 4 0,0:::
```

It is best to erase all LEDs that you do not have connected.

### Modes

#### Warning

This mode simply uses the LEDs to flash when warnings occur.

| Warning | LED Pattern | Notes |
|---------|-------------|-------|
| Arm-lock enabled | flash between green and off | occurs during calibration or when unarmed and the aircraft is tilted too much |
| Low Battery | flash red and off | battery monitoring must be enabled.  May trigger temporarily under high-throttle due to voltage drop |
| Failsafe | flash between light blue and yellow | Failsafe must be enabled |

Flash patterns appear in order, so that it's clear which warnings are enabled.

#### GPS state

This mode shows the GPS state and satellite count.

No fix = red LED
3D fix = green LED

The LEDs will blink as many times as the satellite count, then pause and start again.

#### RSSI level

This mode binds the LED color to RSSI level.

| Color      |   RSSI   |
| ---------- | ---------|
| Green      |   100%   |
| Lime green |    80%   |
| Yellow     |    60%   |
| Orange     |    40%   |
| Red        |    20%   |
| Deep pink  |     0%   |

When RSSI is below 50% is reached, LEDs will blink slowly, and they will blink fast when under 20%.


#### Battery level

This mode binds the LED color to remaining battery capacity.

| Color      | Capacity |
| ---------- | ---------|
| Green      |   100%   |
| Lime green |    80%   |
| Yellow     |    60%   |
| Orange     |    40%   |
| Red        |    20%   |
| Deep pink  |     0%   |

When Warning or Critical voltage is reached, LEDs will blink slowly or fast.
Note: this mode requires a current sensor. If you don't have the actual device you can set up a virtual current sensor (see [Battery](Battery.md)).

#### Blink

This overlay makes the LED blink from the current active color to the alternate color. You can set the blink rate using `ledstrip_blink_period_ms`.

#### Flicker

This overlay makes the LED flicker, a bit like a candle. Set the flicker rate using `ledstrip_flicker_rate`.

#### Fade to alt color

A LED with this overlay will fade to the alternate color in the profile STATUS_ALT. Specify Black as the alternate color to turn LEDs of (e.g. for switching off landing lights). You can set the fade rate with `ledstrip_fade_rate`.

#### Larson Scanner (Cylon Effect)

The Larson Scanner replicates the scanning "eye" effect seen on the mechanical Cylons and on Kitt from Knight Rider.
This overlay dims all of the LEDs it is assigned to and brightens certain ones at certain times in accordance with the animation. The animation is active regardless of arm state.

#### VTX Frequency

This overlay makes the LED color dependent on the current channel of the VTX, in case it is equipped with SmartAudio or IRC Tramp.
The color is selected according to the following table:

 Frequency range | Default color | Color index
 --- | --- | ---
 <= 5672 | White | 1
 |> 5672 <= 5711 | Red | 2
 |> 5711 <= 5750 | Orange | 3
 |> 5750 <= 5789 | Yellow | 4
 |> 5789 <= 5829 | Green | 6
 |> 5829 <= 5867 | Blue | 10
 |> 5867 <= 5906 | Dark violet | 11
 |> 5906 | Deep pink | 13

The default color can be changed by double-clicking the color and moving the Hue slider or by using the color command in the CLI.

#### Flight Mode & Orientation

This mode shows the flight mode and orientation.

When flight modes are active then the LEDs are updated to show different colors depending on the mode, placement on the grid and direction.

LEDs are set in a specific order:
 * LEDs that marked as facing up or down.
 * LEDs that marked as facing west or east AND are on the west or east side of the grid.
 * LEDs that marked as facing north or south AND are on the north or south side of the grid.

That is, south facing LEDs have priority.

The mapping between modes LED placement and colors is currently fixed and cannot be changed.

#### Indicator

This mode flashes LEDs that correspond to roll and pitch stick positions.  i.e.  they indicate the direction the craft is going to turn.

| Mode | Direction | LED Color |
|------------|--------|---------------------|
|Orientation | North  | WHITE			|
|Orientation | East   | DARK VIOLET	|
|Orientation | South  | RED			|
|Orientation | West   | DEEP PINK		|
|Orientation | Up     | BLUE			|
|Orientation | Down   | ORANGE		|
| | | |
|Head Free   | North  | LIME GREEN 	|
|Head Free   | East   | DARK VIOLET 	|
|Head Free   | South  | ORANGE 		|
|Head Free   | West   | DEEP PINK 	|
|Head Free   | Up     | BLUE 			|
|Head Free   | Down   | ORANGE 		|
| | | |
|Horizon     | North  | BLUE			|
|Horizon     | East   | DARK VIOLET 	|
|Horizon     | South  | YELLOW 		|
|Horizon     | West   | DEEP PINK 	|
|Horizon     | Up     | BLUE 			|
|Horizon     | Down   | ORANGE 		|
| | | |
|Angle       | North  | CYAN			|
|Angle       | East   | DARK VIOLET 	|
|Angle       | South  | YELLOW 		|
|Angle       | West   | DEEP PINK 	|
|Angle       | Up     | BLUE 			|
|Angle       | Down   | ORANGE 		|
| | | |
|Mag         | North  | MINT GREEN	|
|Mag         | East   | DARK VIOLET 	|
|Mag         | South  | ORANGE 		|
|Mag         | West   | DEEP PINK 	|
|Mag         | Up     | BLUE 			|
|Mag         | Down   | ORANGE 		|
| | | |
|Baro        | North  | LIGHT BLUE 	|
|Baro        | East   | DARK VIOLET 	|
|Baro        | South  | RED 			|
|Baro        | West   | DEEP PINK 	|
|Baro        | Up     | BLUE 			|
|Baro        | Down   | ORANGE 		|

#### Armed state

This mode toggles LEDs between green and blue when disarmed and armed, respectively. If arming is disabled the LED will be set to the background color, which usually is black.

Note: Armed State cannot be used with Flight Mode.

#### Thrust state

This mode fades the current LED color to the previous/next color in the HSB color space depending on throttle stick position.  When the throttle is in the middle position the color is unaffected, thus it can be mixed with orientation colors to indicate orientation and throttle at the same time.  Thrust should normally be combined with Color or Mode/Orientation.

#### Thrust ring state

This mode is allows you to use one or multiple LED rings (e.g. NeoPixel ring) for an afterburner effect. LEDs with this mode will light up with their assigned color in a repeating sequence. Assigning the color black to an LED with the ring mode will prevent the LED from lighting up.

A better effect is achieved when LEDs configured for thrust ring have no other functions.

LED direction and X/Y positions are irrelevant for thrust ring LED state.  The order of the LEDs that have the state determines how the LED behaves, and the throttle value determines the animation rate. The animation is only active while armed.

Each LED of the ring can be a different color. The color can be selected between the 16 colors available.

For example, LED 0 is set as a `R`ing thrust state LED in color 13 as follow.

```
led 0 2,2::R:13
```

LED strips and rings can be combined.

#### Solid Color

The mode allows you to set an LED to be permanently on and set to a specific color.

x,y position and directions are ignored when using this mode.

Other modes will override or combine with the color mode.

For example, to set LED 0 to always use color 10 you would issue this command.

```
led 0 0,0::C:10
```

### Colors

Colors can be configured using the cli `color` command.

The `color` command takes either zero or two arguments - an zero-based color number and a sequence which indicates pair of hue, saturation and value (HSV).

See [HSL and HSV](http://en.wikipedia.org/wiki/HSL_and_HSV)

If used with zero arguments it prints out the color configuration which can be copied for future reference.

The default color configuration is as follows:

| Index | Color       |
| ----- | ----------- |
|     0 | black       |
|     1 | white       |
|     2 | red         |
|     3 | orange      |
|     4 | yellow      |
|     5 | lime green  |
|     6 | green       |
|     7 | mint green  |
|     8 | cyan        |
|     9 | light blue  |
|    10 | blue        |
|    11 | dark violet |
|    12 | magenta     |
|    13 | deep pink   |
|    14 | black       |
|    15 | black       |

```
color 0 0,0,0
color 1 0,255,255
color 2 0,0,255
color 3 30,0,255
color 4 60,0,255
color 5 90,0,255
color 6 120,0,255
color 7 150,0,255
color 8 180,0,255
color 9 210,0,255
color 10 240,0,255
color 11 270,0,255
color 12 300,0,255
color 13 330,0,255
color 14 0,0,0
color 15 0,0,0
```

### Mode Colors Assignement

Mode Colors can be configured using the cli `mode_color` command.

- No arguments: lists all mode colors
- arguments: mode, function, color

First 8 groups of ModeIndexes are :

| mode | name        |
|------|-------------|
| 0    | orientation |
| 1    | headfree    |
| 2    | horizon     |
| 3    | angle       |
| 4    | mag         |
| 5    | baro        |
| 6    | special     |
| 7    | channel     |

Modes 0 to 5 functions:

| function | name  |
|----------|-------|
| 0        | north |
| 1        | east  |
| 2        | south |
| 3        | west  |
| 4        | up    |
| 5        | down  |

Mode 6 use these functions:

| function | name               |
|----------|--------------------|
| 0        | disarmed           |
| 1        | armed              |
| 2        | animation          |
| 3        | background         |
| 4        | blink background   |
| 5        | gps: no satellites |
| 6        | gps: no fix        |
| 7        | gps: 3D fix        |

The ColorIndex is picked from the colors array ("palette").

Mode 7 is used along with Thrust state to make the LED color dependent on a channel different from the throttle.

Examples (using the default colors):

- set armed color to red: ```mode_color 6 1 2```
- set disarmed color to yellow: ```mode_color 6 0 4```
- set Headfree mode 'south' to Cyan: ```mode_color 1 2 8```
- set color dependent on AUX 1 in Thrust state: ```mode_color 7 0 4```

## Positioning

Cut the strip into sections as per diagrams below.  When the strips are cut ensure you reconnect each output to each input with cable where the break is made. e.g. connect 5V out to 5V in, GND to GND and Data Out to Data In.

Orientation is when viewed with the front of the aircraft facing away from you and viewed from above.

### Example 12 LED config

The default configuration is as follows
```
led 0 15,15:ES:IA:0
led 1 15,8:E:WF:0
led 2 15,7:E:WF:0
led 3 15,0:NE:IA:0
led 4 8,0:N:F:0
led 5 7,0:N:F:0
led 6 0,0:NW:IA:0
led 7 0,7:W:WF:0
led 8 0,8:W:WF:0
led 9 0,15:SW:IA:0
led 10 7,15:S:WF:0
led 11 8,15:S:WF:0
led 12 7,7:U:WF:0
led 13 8,7:U:WF:0
led 14 7,8:D:WF:0
led 15 8,8:D:WF:0
led 16 8,9::R:3
led 17 9,10::R:3
led 18 10,11::R:3
led 19 10,12::R:3
led 20 9,13::R:3
led 21 8,14::R:3
led 22 7,14::R:3
led 23 6,13::R:3
led 24 5,12::R:3
led 25 5,11::R:3
led 26 6,10::R:3
led 27 7,9::R:3
led 28 0,0:::0
led 29 0,0:::0
led 30 0,0:::0
led 31 0,0:::0
```

Which translates into the following positions:

```
     6             3
      \           /
       \   5-4   /
        \ FRONT /
    7,8 | 12-15 | 1,2
        /  BACK \
       /  10,11  \
      /           \
     9             0
       RING 16-27
```

LEDs 0,3,6 and 9 should be placed underneath the quad, facing downwards.
LEDs 1-2, 4-5, 7-8 and 10-11 should be positioned so the face east/north/west/south, respectively.
LEDs 12-13 should be placed facing down, in the middle
LEDs 14-15 should be placed facing up, in the middle
LEDs 16-27 should be placed in a ring and positioned at the rear facing south.

This is the default so that if you don't want to place LEDs top and bottom in the middle just connect the first 12 LEDs.

### Example 16 LED config

```
led 0 15,15:SD:IA:0
led 1 8,8:E:FW:0
led 2 8,7:E:FW:0
led 3 15,0:ND:IA:0
led 4 7,7:N:FW:0
led 5 8,7:N:FW:0
led 6 0,0:ND:IA:0
led 7 7,7:W:FW:0
led 8 7,8:W:FW:0
led 9 0,15:SD:IA:0
led 10 7,8:S:FW:0
led 11 8,8:S:FW:0
led 12 7,7:D:FW:0
led 13 8,7:D:FW:0
led 14 7,7:U:FW:0
led 15 8,7:U:FW:0
```

Which translates into the following positions:

```
     6             3
      \           /
       \   5-4   /
      7 \ FRONT / 2
        | 12-15 |
      8 /  BACK \ 1
       /  10-11  \
      /           \
     9             0
```

LEDs 0,3,6 and 9 should be placed underneath the quad, facing downwards.
LEDs 1-2, 4-5, 7-8 and 10-11 should be positioned so the face east/north/west/south, respectively.
LEDs 12-13 should be placed facing down, in the middle
LEDs 14-15 should be placed facing up, in the middle

### Example 28 LED config

```
#right rear cluster
led 0 9,9:S:FWT:0
led 1 10,10:S:FWT:0
led 2 11,11:S:IA:0
led 3 11,11:E:IA:0
led 4 10,10:E:AT:0
led 5 9,9:E:AT:0
# right front cluster
led 6 10,5:S:F:0
led 7 11,4:S:F:0
led 8 12,3:S:IA:0
led 9 12,2:N:IA:0
led 10 11,1:N:F:0
led 11 10,0:N:F:0
# center front cluster
led 12 7,0:N:FW:0
led 13 6,0:N:FW:0
led 14 5,0:N:FW:0
led 15 4,0:N:FW:0
# left front cluster
led 16 2,0:N:F:0
led 17 1,1:N:F:0
led 18 0,2:N:IA:0
led 19 0,3:W:IA:0
led 20 1,4:S:F:0
led 21 2,5:S:F:0
# left rear cluster
led 22 2,9:W:AT:0
led 23 1,10:W:AT:0
led 24 0,11:W:IA:0
led 25 0,11:S:IA:0
led 26 1,10:S:FWT:0
led 27 2,9:S:FWT:0
```

```
       16-18  9-11
19-21 \           / 6-8
       \  12-15  /
        \ FRONT /
        /  BACK \
       /         \
22-24 /           \ 3-5
       25-27   0-2
```

All LEDs should face outwards from the chassis in this configuration.

Note:
This configuration is specifically designed for the [Alien Spider AQ50D PRO 250mm frame](http://www.goodluckbuy.com/alien-spider-aq50d-pro-250mm-mini-quadcopter-carbon-fiber-micro-multicopter-frame.html).


## Troubleshooting

On initial power up the LEDs on the strip will be set to WHITE.  This means you can attach a current meter to verify the current draw if your measurement equipment is fast enough.  Most 5050 LEDs will draw 0.3 Watts a piece.
This also means that you can make sure that each R,G and B LED in each LED module on the strip is also functioning. After a short delay the LEDs will show the unarmed color sequence and or low-battery warning sequence.

Also check that the feature `LED_STRIP` was correctly enabled and that it does not conflict with other features, as above.
