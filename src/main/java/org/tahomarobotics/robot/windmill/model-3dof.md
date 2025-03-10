## Elevator-Arm Dynamics

### 1. Define the System Coordinates

We have two degrees of freedom:
* $x$: Mobile base position of the robot. Right is positive.
* $y$: Vertical position of the elevator. Up is positive from zero at bottom.
* $\theta$: Rotation angle of the arm. Rotating counter-clockwise from zero at horizontal right.

The state variables are:
* $x, \dot x, \ddot x$ (position, velocity and acceleraation of the robot base)
* $y,\dot y,\ddot y$ (position, velocity and acceleration of the elevator)
* $\theta,\dot\theta,\ddot\theta$ (position, velocity and accleration of the arm)

### 2. Kinetic Energy (T)
**Robot Base**
The robot's kinetic energy comes from it horizontal motion:
$$T_{base} = \dfrac12m_b\dot x^2$$  
**Elevator**  
The elevator's kinetic energy comes from its vertical motion:
$$T_{elevator}=\dfrac12(m_e\dot x^2+m_e\dot y^2)$$
**Arm**  
The arm's kinetic energy has two components (rotational and translation of the center of mass):
$$T_{arm} = \dfrac12m_av_a^2 + \dfrac12I\theta^2 = \dfrac12m_a(v_{ax}^2+v_{ay}^2) + \dfrac12I\theta^2$$  
Differentiating the position x and y gets the velocity of the center of mass:  
$$v_{ax}=\dfrac{\mathrm{d}}{\mathrm{dt}}\left(x+l_ccos\theta\right)=\dot x-l_c\dot\theta sin\theta \quad v_{ay}=\dfrac{\mathrm{d}}{\mathrm{dt}}\left(y + l_csin\theta\right)=\dot y + l_c\dot\theta cos\theta$$  
Factoring that component velocity back in and expanding:
$$T_{arm} = \dfrac12m_a((\dot x-l_c\dot\theta sin\theta)^2+(\dot y + l_c\dot\theta cos\theta)^2) + \dfrac12I\theta^2$$
$$T_{arm} = \dfrac12m_a(\dot x^2 -2\dot xl_c\dot\theta sin\theta +l_c^2\dot\theta^2 sin^2\theta+\dot y^2 + 2\dot yl_c\dot\theta cos\theta + l_c^2\dot\theta^2 cos^2\theta) + \dfrac12I\theta^2$$  
Then reducing terms with $cos^2\theta+sin^2\theta=1$:
$$T_{arm} = \dfrac12m_a(\dot x^2 - 2\dot xl_c\dot\theta sin\theta + \dot y^2 + 2\dot yl_c\dot\theta cos\theta + l_c^2\dot\theta^2) + \dfrac12I\theta^2$$  
**Total Kinetic Energy**  
The total kinetic energy is:
$$T=T_{base}+T_{elevator}+T_{arm}$$  
Substitute the terms:
$$T=\dfrac12m_b\dot x^2+\dfrac12(m_e\dot x^2+m_e\dot y^2)+\dfrac12m_a\left(\dot x^2 + \dot y^2-2\dot xl_c\dot\theta sin\theta+2\dot yl_c\dot\theta cos\theta + l_c^2\dot\theta^2\right)+\dfrac12I\dot \theta^2$$
Simplify:
$$T=\dfrac12(m_b+m_e+m_a)\dot x^2+\dfrac12(m_e+m_a)\dot y^2-m_a\dot xl_c\dot\theta sin \theta+m_a\dot yl_c\dot\theta cos\theta + \dfrac12\left(m_al_c^2+I\right)\dot \theta^2$$
### 3. Potential Energy (V)  
**Robot Base**  
The robot base's potential energy is zero as it has no change in height:
$$V_{base}=0$$
**Elevator**  
The elevator's potential energy is due to its height:  
$$V_{elevator}=m_egy$$
**Arm**  
The arm's potential energy is due to the height of its center of mass:  
$$V_{arm}=m_ag(y+l_csin\theta)$$
**Total Potential Energy**  
$$V=V_{base}+V_{elevator}+V_{arm}=m_egy+m_ag(y+l_csin\theta)$$  
### 4. The Lagrangian  
The Lagrangian is:
$$\mathcal{L}=T-V$$  
Substituting $T$ and $V$:
$$\mathcal{L}=\dfrac12(m_b+m_e+m_a)\dot x^2+\dfrac12(m_e+m_a)\dot y^2 - m_a\dot xl_c\dot\theta sin\theta +m_a\dot yl_c\dot\theta cos\theta + \dfrac12\left(m_al_c^2+I\right)\dot \theta^2 - (m_egy+m_ag(y+l_csin\theta))$$  
### 5. Equations of Motion  
Using the **Euler-Lagrange Equation**:  
$$\dfrac{\mathrm{d}}{\mathrm{dt}}\left(\dfrac{\partial\mathcal{L}}{\partial{\dot q_i}}\right)-\dfrac{\partial\mathcal{L}}{\partial{q_i}}=Q_i$$  
Where $q_i$ are the generalized coordinates $(x,y\ and\ \theta)$, $Q_i$ are the generalized forces.  

**Generalized Coordinate $x$ (Robot Base):**
1. Compute $\dfrac{\partial\mathcal{L}}{\partial\dot x}$:
   $$\dfrac{\partial\mathcal{L}}{\partial\dot x}=(m_b+m_e+m_a)\dot x-m_al_c\dot\theta sin\theta$$
2. Compute $\dfrac{\mathrm{d}}{\mathrm{dt}}\left(\dfrac{\partial\mathcal{L}}{\partial\dot x}\right)$:  
   $$\dfrac{\mathrm{d}}{\mathrm{dt}}\left(\dfrac{\partial\mathcal{L}}{\partial\dot x}\right) = (m_b+m_e+m_a)\ddot x-m_al_c\left(\ddot\theta sin\theta+\dot\theta^2 cos\theta\right)$$
3. Compute $\dfrac{\partial\mathcal{L}}{\partial x}$:  
   $$\dfrac{\partial\mathcal{L}}{\partial x}=0$$  
   Combine to get the equation of motion for $x$ and include motor force:
   $$(m_b+m_e+m_a)\ddot x-m_al_c\left(\ddot\theta sin\theta+\dot\theta^2 cos\theta\right) = F_{base}$$

**Generalized Coordinate $y$ (Elevator):**  
1. Compute $\dfrac{\partial\mathcal{L}}{\partial\dot y}$: 
$$\dfrac{\partial\mathcal{L}}{\partial\dot y}=(m_e+m_a)\dot y+m_al_c\dot\theta cos\theta$$
2. Compute $\dfrac{\mathrm{d}}{\mathrm{dt}}\left(\dfrac{\partial\mathcal{L}}{\partial\dot y}\right)$:  
$$\dfrac{\mathrm{d}}{\mathrm{dt}}\left(\dfrac{\partial\mathcal{L}}{\partial\dot y}\right) = (m_e+m_a)\ddot y+m_al_c\left(\ddot\theta cos\theta-\dot\theta^2 sin\theta\right)$$
3. Compute $\dfrac{\partial\mathcal{L}}{\partial y}$:  
$$\dfrac{\partial\mathcal{L}}{\partial y}=-\left(m_e+m_a\right)g$$  
Combine to get the equation of motion for $y$ and include motor force:  
$$(m_e+m_a)\ddot y+m_al_c\left(\ddot\theta cos\theta-\dot\theta^2 sin\theta\right) + \left(m_e+m_a\right)g = F_{motor}$$  

**Generalized Coordinate $\theta$ (Arm):**
1. Compute $\dfrac{\partial\mathcal{L}}{\partial\dot\theta}$:
   $$\dfrac{\partial\mathcal{L}}{\partial\dot \theta}=-m_a\dot xl_c sin\theta + m_a\dot yl_c cos\theta + \left(m_al_c^2+I\right)\dot \theta $$
2. Compute $\dfrac{\mathrm{d}}{\mathrm{dt}}\left(\dfrac{\partial\mathcal{L}}{\partial\dot \theta}\right)$:  
   $$\dfrac{\mathrm{d}}{\mathrm{dt}}\left(\dfrac{\partial\mathcal{L}}{\partial\dot \theta}\right) = -m_al_c \ddot xsin\theta-m_al_c\dot\theta \dot xcos\theta + m_al_c \ddot ycos\theta-m_al_c\dot\theta \dot ysin\theta + \left(m_al_c^2+I\right)\ddot \theta $$
3. Compute $\dfrac{\partial\mathcal{L}}{\partial\theta}$:  
   $$\dfrac{\partial\mathcal{L}}{\partial\theta}=-m_a\dot xl_c\dot\theta cos\theta-m_a\dot yl_c\dot\theta sin\theta - m_agl_ccos\theta$$  
   Combine to get the equation of motion for $\theta$ and include motor torque:  
   $$-m_al_c\ddot x sin\theta-m_al_c\dot\theta\dot xcos\theta+m_al_c \ddot ycos\theta-m_al_c\dot\theta \dot ysin\theta + \left(m_al_c^2+I\right)\ddot \theta + m_a\dot xl_c\dot\theta cos\theta+ m_a\dot yl_c\dot\theta sin\theta + m_agl_ccos\theta = \tau_{motor}$$  
   $$-m_al_c\ddot x sin\theta+m_al_c \ddot ycos\theta + \left(m_al_c^2+I\right)\ddot \theta + m_agl_ccos\theta = \tau_{motor}$$  

### 6. Matrix form
Starting with:
$$(m_b+m_e+m_a)\ddot x-m_al_c\left(\ddot\theta sin\theta+\dot\theta^2 cos\theta\right) = F_{base}$$
$$(m_e+m_a)\ddot y+m_al_c\left(\ddot\theta cos\theta-\dot\theta^2 sin\theta\right) + \left(m_e+m_a\right)g = F_{motor}$$
$$-m_al_c\ddot x sin\theta+m_al_c \ddot ycos\theta + \left(m_al_c^2+I\right)\ddot \theta + m_agl_ccos\theta = \tau_{motor}$$  
Let's organize the equations of motion into the canonical form:
$$M\ddot q +C(q,\dot q)\dot q + G(q) = \tau$$
1. **Mass Matrix, $M$:**
$$M = \begin{bmatrix}m_b+m_e+m_a&0&  -m_al_c sin\theta \\0&m_e+m_a&m_al_c cos\theta \\-m_al_csin\theta&m_al_ccos\theta&m_al_c^2+I\end{bmatrix}$$
2. **Velocity Product Terms, $C(q,\dot q)$**
$$C(q,\dot q) = \begin{bmatrix}-m_al_c\dot\theta^2 cos\theta\\-m_al_c\dot\theta^2 sin\theta\\0\end{bmatrix}$$
3. **Gravity Terms, $G(q)$:**
$$G(q) = \begin{bmatrix}0\\\left(m_e+m_a\right)g\\m_agl_ccos\theta\end{bmatrix}$$
4. **Generalized Force/Control Input, $\tau$:**
$$\tau = \begin{bmatrix}F_{base}\\F_{motor}\\\tau_{motor}\end{bmatrix}$$
