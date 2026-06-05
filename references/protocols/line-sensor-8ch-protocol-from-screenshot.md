# Eight-Channel Line Sensor Protocol

Source: user-provided screenshot in this conversation.

## Supported Interfaces

- IO direct level read: returns digital channel values.
- USART: returns digital values, analog values, or both.
- I2C: returns digital values only.

## USART Command Format

Host command:

```text
$<calibrate>,<analog_enable>,<digital_enable>#
```

Fields:

- `calibrate`: calibration switch.
- `analog_enable`: analog data send switch.
- `digital_enable`: digital data send switch.

Examples:

```text
$1,0,0#
```

Calibration command. The screenshot says this only enters calibration; reading black/white values depends on the board button.

```text
$0,0,1#
```

Request digital data.

```text
$0,1,0#
```

Request analog data.

```text
$0,1,1#
```

Request digital and analog data.

The module does not send data by default after power-up. It only continuously sends the corresponding data after receiving the matching command.

## USART Digital Response

```text
$D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#
```

The current verified hardware reports black line as `0` and white background as `1`. The firmware therefore treats every zero channel value as an active line bit. If a future module uses the opposite polarity, use the live parameter `sensor.invert_bits`.

## USART Analog Response

```text
$A,x1:4096,x2:4096,x3:4096,x4:4096,x5:4096,x6:4096,x7:4096,x8:4096#
```

Analog values are not used in the first digital-only line-following scaffold.
