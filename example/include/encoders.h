#ifndef ENCODERS_H
#define ENCODERS_H

#define COUNTS_PER_MOTOR_REV (12)
#define GEAR_RATIO (15.25)
#define COUNTS_PER_REV (COUNTS_PER_MOTOR_REV * GEAR_RATIO)
#define SECONDS_PER_MIN (60)
#define MEASURE_PERIOD_SEC (1)
#define SEC_TO_uS (1000000)

#define WHEEL_DIAMETER_MM (32.5)
#define WHEEL_CIRCUMFERENCE_MM (106.81)

#define CALIB (1)

#define GPIO_IRQ_MASK (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)

#define RIGHT_ENCODER_A_PIN (12)
#define RIGHT_ENCODER_B_PIN (13)

#define LEFT_ENCODER_A_PIN (2)
#define LEFT_ENCODER_B_PIN (3)

void encoders_register_callbacks(void);

void encoders_zero_distances(void);

float encoders_distance_traveled(void);

bool timer_callback(repeating_timer_t *rt);

#endif // ENCODERS_H

