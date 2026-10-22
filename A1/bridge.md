# Bridge Test Decision

I implemented the Bridge Test as an optional sampling strategy with
`bridge_prob = 0.3`, meaning that 30% of sampling attempts use the Bridge
Test and the remaining 70% use uniform sampling.

However, the experiments did not demonstrate that this option performed
consistently better on my test scenarios:

| Environment | Uniform sampling | Bridge Test (`0.3`) |
|---|---:|---:|
| E5 severe bottleneck | 20/25 | 21/25 |
| E6 multiple gaps | 25/25 | 25/25 |
| E7 offset passages | 1/15 | 0/15 |

The Bridge Test slightly improved E5, made no difference to E6's success
rate, and performed worse on E7. These results are insufficient to establish
a reliable improvement. Bridge attempts also consume part of the finite
20-second sampling budget.

Therefore, I retained uniform sampling as the default method and disabled the
Bridge Test by default. The implementation remains available as an optional
experiment, but it is not part of the baseline planner described in B2 or used
in the default B3 demonstration.

## Oral Answer

> I implemented the Bridge Test with a 30% sampling probability, but my
> experiments did not show that it consistently outperformed uniform
> sampling on the tested scenarios. Because Bridge Test attempts consume
> additional time within the fixed planning budget, I kept uniform sampling
> as the default and left the Bridge Test disabled.
