# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## The effect of the P, I, D components:

- P component:
The P component helps to remove the main portion of error(cte). If the car starts to move away from the middle of the road, the controller corrects it's movement by adjusting the steering angle proportionally to the P component.

- D component:
Without the D component our car will wobble around the middle of the road. The controller will infinitely send commands to steer right and left. D component helps to keep the car move straight.

- I component:
Since the car doesn't have systematic bias, the P and D components are enough. The high value of the I-component's value affects negatively and prevents the controller to converge at start. However the low value of I-coefficient in my experience makes the car move more smoothly.

## Choosing the hyperparameters.

The coefficients for P, I and D components were choosen by applying the Twiddle algorithm. The initial values for deltas were choosen by manual tuning to make it converge faster.
To help the algorithm converge at start, the throttle is set to low value and increases later. 

