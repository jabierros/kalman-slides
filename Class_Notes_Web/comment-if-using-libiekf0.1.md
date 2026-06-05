### If using `LibIEKF0` data generation capabilities
We need to define the function $\frac{d~\mathbf{x}}{d~t}(\mathbf{x},\mathbf{u})$,  defining the _true_ differential equation for the _true_ state
$$\frac{d~\mathbf{x}}{d~t}(\mathbf{x},\mathbf{u})=\dot{\mathbf{x}}(\mathbf{x},\mathbf{u})
\equiv
\dot{\mathbf{x}}^{\text{true fun}}(\mathbf{x},\mathbf{u})$$
its initial state $\mathbf{x}_0$, and the function that defines the _true input_, $\mathbf{u}$,  in terms of time, $t$:
$$\mathbf{u}=\mathbf{u}^{\text{true fun}}(t)$$
 the true measurements are in turn defined by $\mathbf{h}^{\text{true}}(\mathbf{x},\mathbf{u},t)$:
  $$\mathbf{z}^{\text{true fun}}(t)=\mathbf{h}^{\text{true}}({\mathbf{x}}^{\text{true fun}}(t),\mathbf{u}^{\text{true fun}}(t),t)$$

 These functions are used to generate _true state_ and _true measurements_ (for sensors an inputs). From these, real measurements are defined by adding true noise, that is generated through user defined functions:
  $$\mathbf{u}^{\text{meas fun}}(t)=\mathbf{u}^{\text{true fun}}(t)+\mathbf{w}_{\mathbf{u}}^{\text{true fun}}(t)$$$$\mathbf{z}^{\text{meas fun}}(t)=\mathbf{z}^{\text{true fun}}(t)+\mathbf{w}_{\mathbf{z}}^{\text{true fun}}(t)$$
These true noise functions are driven from a source of white noise.
