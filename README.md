# MPU6050_DMP6_Multiple
I will move this to https://github.com/jrowberg/i2cdevlib once this example is tested.

This example connects to two MPU6050. But it is possible to extend the example to connect to any number of accelerometers. Only the number of interrupt pins sets the limit.

~~In the unaltered example you need to connect your MPU6050 interrupt pin to Arduino like this:~~
```
#define INTERRUPT_PIN_0 2  // use pin 2 on Arduino Uno & most boards to INT pin on IMU #0
#define INTERRUPT_PIN_1 3  // use pin 3 on Arduino Uno & most boards to INT pin on IMU #1
```

You will also need to connect Arduino output pins to each MPU6050 AD0 pin:
```
#define AD0_PIN_0 4  // Connect this pin to the AD0 pin on IMU #0
#define AD0_PIN_1 5  // Connect this pin to the AD0 pin on IMU #1
```

To set the FIFO rate with the `MPU6050_DMP_FIFO_RATE_DIVISOR` macro you need to checkout the develop branch of i2cdevlib:
```
cd i2cdevlib
git checkout develop
```
