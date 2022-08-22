# MPC based on OSQP
* This project gives an example, which can be modified by users.    
* Every block during option start and option end should be modified by users according to the actual situation, where contains printing options and necessary parameters.   
* Need to pre install dependency `Eigen`:    
  `sudo apt-get install libeigen3-dev`      
  `sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen`  
* Need to pre install dependency `OSQP-0.4.1`, which can be get from:   
  [OSQP](`https://github.com/osqp/osqp`)  
* Provide simple parameter type checking function.  
* The discrete form of the state space equation should be given by users, this project does not provide form transformation function similar to that in project [DLQR](https://github.com/Technician13/DLQR).       
* In `main` function, a loop is used to simulate the normal operation of a classical 1-dim position-velocity-acceleration system, the period of sampling is set to `0.5` and the number of iterations is `20`.     

$$\begin{bmatrix}p_{k+1}\\\\v_{k+1}\end{bmatrix} =
  \begin{bmatrix}1 & T\\\\0 & 1\end{bmatrix} \cdot \begin{bmatrix}p_{k}\\\\v_{k}\end{bmatrix} + 
  \begin{bmatrix}\frac{1}{2} \cdot T^2 \\\\T\end{bmatrix} \cdot u_k$$    

* Specific derivation process can be seen in [Appendix A](#appendix-a), all variables' names are the same as those in the code.  
* Prediction time domain `horizon` of MPC is set to 3, so the weight matrices `H` nad `g` are simply formatted as identity matrices as follows:       
$$\boldsymbol{H} =  \boldsymbol{1}_{6\times6}$$
$$\boldsymbol{g} =  \boldsymbol{1}_{3\times3}$$
 

* The test result is as the follow  figure, as the time going, the pos in blue converge to `5` from the initial state `2` and the vel in red converge to `0` from the initial state `0`:        


<div align=center>
  <img src="img/system.png" width=60%>
</div>

* The follow figure display the change process of the input of the whole system, in this sample, the input is limited between `-2` and `2`.

<div align=center>
  <img src="img/control.png" width=60%>
</div>

</br>



# Appendix A    
* Sorry, for some special reasons, the formula markdown source code of the derivation process is not open source for the time being.        

<div align=center>
  <img src="img/derivation.png">
</div>

</br>   



# Tips    
* A larger `horizon` will make the control effect better after referring to more prediction information. However, the phenomenon that the scale of the problem increases must also be taken into consideration. Therefore, a suitable `horizon` is far more important than a larger `horizon`.      
* The elements in the weight matrix `H` are used to specify the system variables that are more concerned in the calculation process. However, due to the coupling relationship between the system variables, change the value of one element in `H` will affect the control effect of the controller on other system variables. Therefore, be careful when tuning `H`.      
* The example in this project provides a simple and rough $\boldsymbol{X}_{ref}$ selection method. In order to achieve better control effect, a more flexible method should be selected in practical application.      

