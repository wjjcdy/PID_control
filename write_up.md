# The effect of P,I,D component
1. I can set P component with any value. If I set P value very bigger. The car would turn the middle of the road fastly. If P value is smaller, the car turn slowly. The value of P can affect the speed of the car turn the middle. 
2. If I only use P component used for controller. The car will overshoot the middle of the road. I added D component and set D with a value. The car will just overshoot less than no D component. The car would overshoot very little by adjusting the value of the D many times manually.
3. If the steer of the car have systematic bias.I test many times by setting I component. I componet must be a very small value. When the I coefficients, the car will leave the drivable portion of the track surface. I think the systematic bias of this car should be small.

# how I chose the final hyperparameters (P, I, D coefficients)

At first, I want to get the best P, I, D coefficients with the twiddle method. But the car can't track the way when optimizating. I know the reason because the when P, I, D coefficients is not OK when optimizating. But I don't know how to do it.

Finially, I tuned hyperparameter manually. I tuned the P, I, D coefficients based on their function. At first, I set I and D coefficients is 0. I just tuned the P coefficients. If the car can't turn the middle when the speed of the car is high. I tuned bigger. If the car would turn the middle when the speed of the car is high. But the car overshoot the middle. I tuned the D coefficients bigger. After many times, I found some good values.  P coefficients is 0.15.  I coefficients is 0.001.  D coefficients is 1.5. 