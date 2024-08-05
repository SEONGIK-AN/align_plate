## 칼만필터를 자이로-가속도 센서에 적용하기 위한 방법
### **개념설명**
> #### 요약
> - 칼만 필터의 알고리즘은 **예측과 업데이트**를 통해 이루어진다.
> - **예측 단계**에서는 이전 스텝에서의 상태 추정치를 사용하여 현재 스텝의 상태 추정치를 생성한다.
> - 이 때, 칼만 필터의 현재 상태 추정치는 현 상태의 측정치를 사용하지 않으므로 선험적 상태 추정치이다.
> - **업데이트 단계**에서는 예측 상태에서 구한 추정치와 관측 정보 간의 차이에 최적 칼만 이득을 곱함으로써 이전 상태 추정치와 결합하여 상태 추정치를 구체화하여 사후 상태 추정치를 구한다. 
#### 칼만 필터란?
- 선형 2차 추정 방법
- 시간이 지남에 따라 관찰된 측정값을 사용하여 미지의 변수의 추정치를 생성하는 알고리즘

#### 계산방법
- 시스템에서 알려진 입력값, 측정값으로부터 추정치를 형성한다.
  → 해당 과정을 통해 하나의 측정값으로부터 얻은 추정치보다 더 나은 값을 산출할 수 있음
- Kalman Filter 사용 데이터⁠
	- 센서 데이터
	- 시스템의 방정식의 근사치
	- 설명 불가한 **외부 요인**
- 계산 시, **가중치**를 사용하여 불확실성이 작은 값을 더 큰 값을 가지도록 한다.
  → 가중치는 **공분산**으로부터 산출

#### 시스템 모델
- 선형 동적  시스템을 기반으로, **Markov 체인**으로써 구축된다.
- 시스템의 상태는 은닉된 실측 시스템 구성을 나타내, 실수의 벡터로 표현된다.
- 마르코프 모델과 유사하나, **연속 공간**에서 값을 가진다는 점이 차이점

#### 시퀀스
- 시간  $k$의 상태  $x_k$는  $(k-1)$ 상태로부터 얻어지며, 이에 대한 공식은 다음과 같다.  
  - $x_k = F_k x_{k-1} + B_k u_k + w_k$  

- 이 때, 각 변수는 다음을 의미 한다.  
	- $F_k$: 이전 상태  $x_{k-1}$에 대한 상태 전이 모델
	- $B_k$:  $u_k$에 대한 컨트롤 입력 모델
	- $w_k$: 프로세스 상 노이즈; 다변량 정규 분포이며,  $Q_k$을 분산으로 가짐.  
  	- $w_k \sim N(0, Q_k)$
	- 시간 k에서, 측정된 $z_k$에 대하여 실제 상태  $x_k$는 다음 식을 따른다.  
  	- ${z_k = H_k x_k + v_k}$
- 이 때, 각 변수는 다음을 의미한다.
- $H_k$: 관측 모델; 실제 상태 공간을 관측 공간에 매핑
- $v_x$: 관측 잡음; 공분산  $R_k$를 가짐
	- $v_k \sim N(0, R_k)$

- $F_k:$  상태전이 모델  
- $H_k$: 관찰 모델  
- $Q_k$: 공정 노이즈의 공분산  
- $R_k$: 관측 노이즈의 공분산  
- $B_k$: 컨트롤-입력 모델; 하기 값을 포함한다.  
- $u_k$: 컨트롤-입력 벡터

### 세부사항
- 칼만 필터는 이전 시간으로부터 추정된 **추정값과 현재 측정값**만을 사용하여 계산하는 재귀 추정기이다. 이에 따라 실제 상태의 추정치는 다음과 같이 표기된다.  
```math
\hat{x}_{n|m}
```  
- 상기값은 $m{\le} n$까지의 관측치를 통해 구해진 $n$일 때의 **추정치**를 표기한 것이다. 
- 필터의 상태는 다음 두 변수로 표시 될 수 있다.
	- ${x_{k|k}}$: k까지 관측치가 주어졌을 때 k에서의 사후적 평균 상태 추정치 (**사후적** 추정치: k까지 관찰이 완료된 후 추정)
	- ${P_{k|k}}$: 사후 추정 공분산 행렬

### 예측 단계
- 예측 단계에서 사용되는 식은 다음과 같다.

| (선험적) 상태 추정치 | ${\hat {x}}_{k {\mid} k-1} = F_{x} x_{k-1 {\mid} k-1} + B_{k} u_{k}$  |
| ------------ | --------------------------------------------------------- |
| (선험적) 추정 공분산 | ${\hat {P}}_{k {\mid} k-1} = F_{x} P_{k-1 {\mid} k-1}F_{k}^{T} + Q_[k]$ |

### 업데이트 단계
- 업데이트 단계에서 사용되는 식은 다음과 같다.

| 추정치 잔차 또는 사전 측정 잔차           | ${\tilde{y}_{k}} = z_{k} - H_{k} {\hat{x}}_{k {\mid} k-1}$            |
| ----------------------------------------- | ------------------------------------------------------- |
| 추정치 잔차(또는 사전 측정 잔차)의 공분산 | $S_{k} = H_{k} {\hat{P}}_{k {\mid} k-1} H_{k}^{T} + R_{k}$             |
| 최적 칼만 이득                            | $K_k = \hat{P}_{k \mid k-1} H_k^T S_k^{-1}$             |
| 업데이트된 (사후) 상태 추정치             | $x_{k \mid k-1} = \hat{x}_{k \mid k-1} + K_k \tilde{y}_k$ |
| 업데이트된 (사후) 추정 공분산             | $P_{k \mid k} = (I - K_k H_k) \hat{P}_{k \mid k-1}$     |
| 측정 후 잔차                              | ${\tilde {y}}_{k \mid k} = z_{k} - H_{k} x_{k {\mid} k}$          |

### **자이로스코프로에의 적용**
#### 시스템 해석
- 앞서 설명한 개념 중 $k$시간에 대한 **실제 시스템의 상태**는 다음과 같이 표현된다.  
```math
x_k = F_k x_{k-1} + B_k u_k + w_k
```  
- 이때, 실제 시스템의 상태는 자이로 센서에 의해 측정되므로 다음 행렬으로 표현될 수 있다.  
```math
x_k = \begin{bmatrix} \theta \\ \dot{\theta}_b \end{bmatrix}_k
```  

- 여기서 $\theta$은 자이로센서에서 취득한 각도값, $\dot{\theta}_b$은 자이로센서에서 측정된 편향을 나타낸다.
- 필터의 출력은 각도이며, 가속도계를 이용할 경우 $\theta$의  편향 값이 발생할 수 있다.
- 실제 속도는 측정값에서 $\dot{\theta}_b$를 뺌으로써 얻을 수 있다.

- 다음은 이전 상태 값 $(x_{k-1})$에 곱해지는 상태 전이 모델 행렬이다.
  - ${ F = \begin{bmatrix} 1 & -\triangle t \\ 0 & 1 \end{bmatrix} }$

- 실제 상태를 자이로스코프 측정치를 적용하여 나타내면 다음과 같다.  
```math
${x_k = F_k x_{k-1} + B \dot{\theta}_k + w_k}$
```  
- 여기서, 제어 입력 모델 $B$는 각속도로부터 각도를 얻어야하고, 편향은 계산할 수 없으므로 다음과 같이 표현된다.
  - ${B = \begin{bmatrix} \triangle t \\ 0 \end{bmatrix}}$
- 상기 상태를 나타난 식에서 노이즈 $w_{k}$는 다음과 같은 값을 따라 정규 분포를 따른다.
  - ${w_{k} \sim N(0, Q_{k})}$
- $Q_{k}$는 노이즈의 공분산 행렬이며, 가속도계와 편향의 상태 추정에 대한 값으로 나타난다. 이 때, 편향과 가속도계의 추정치는 **독립**적인 것으로 간주하므로 가속도계와 편향 추정치의 분산은 동일하다. 따라서, 공분산 행렬은 다음과 같이 정의된다.
  - ${Q_{k} = \begin{bmatrix} Q_{\theta} & 0 \\ 0 & Q_{\dot{\theta}_{b}} \end{bmatrix} \triangle t}$
- 위 식에서 공분산 행렬 $Q_{k}$는 시간 $k$에 따라 달라지므로 각 분산 값에 대하여 시간을 곱하였다. 이 때, 추정된 각도에서 Drift(발산)하기 시작하면 $Q_{\dot{\theta}_b}$ 값을 늘려야하며, 추정이 너무 느릴 경우 각도 추정을 너무 많이 신뢰하는 것이므로 $Q_\theta$값을 줄여야 반응성을 높여야 한다.
#### 측정치 해석
- 측정치$z_k$는 실제 시스템$x_k$에서 측정 오차$v_k$가 생겨 발생하는 것으로 볼 수 있으므로, 다음 식을 따른다.  
```math
{z_k = H_k x_k + v_k}
```  
- 위 식에서 매핑을 위한 항$H$는 측정값은 가속도계로부터의 측정값이므로  다음과 같은 식을 따른다.
```math
{H = \begin{bmatrix}1 & 0\end{bmatrix}}
```  
- 측정 노이즈의 평균은 0이고 공분산 $R$이 존재하는 가우스 분포를 따라야하므로 다음 식을 따른다.
```math
{v_{k} \sim N(0, R)}
```  
- 이때, $R$은 행렬이 아니기 때문에 측정 노이즈는 측정의 분산과 동일하다(동일한 변수의 공분산은 분산과 동일하기 때문) 따라서,  
```math
{R = \begin{bmatrix} v_k & v_k^T\end{bmatrix}=var(v_k)}
```  
- 이때, 측정 노이즈는 동일하고 시간 $k$에 대해 의존하지 않는다고 가정한다.
```math
{var(v_k) =var(v)}
```  
- 상기 식은 필터를 의미하며, 측정 노이즈에 대해 크기를 과도하게 키울 경우 측정에 대한 신뢰가 적어 반응이 느려지며, 너무 작은 경우 가속도계 측정을 너무 신뢰하여 값이 오버슈트될 수 있다.

### **칼만 필터 방정식**
- 상기 과정을 통해 실제 상태를 추정하기 위해 예측과 업데이트 단계를 거쳐 자이로스코프에 칼만필터를 적용할 수 있다.
#### 예측 단계
- 선험적 상태 추정치는 ==**자이로 센서**==측정에 의해 현 상태를 예측하므로, 현 상태의 추정치는 다음 식을 따른다.  
```math
\hat{x}_{k \mid k-1} = F \hat {x}_{k-1 \mid k-1} + B \dot {\theta}_k
```  
- 이때, $B$와 곱해지는 $\hat {\theta}_k$는 시간 $k$일 때의 상태를 추정하기 위해 자이로 센서로부터의 측정값을 추가 입력함으로써 사용되므로 **제어 입력**으로 볼 수 있다.
- 시간$k$에 발생하는 오차의 공분산 추정치는 이전 시간에서 측정된 값을 통해 예측되므로 다음식에 따른다.  
```math
\hat {P}_{k \mid k-1} = F P_{k-1 \mid k-1}F_k^T + Q_k
```  
- 상기 식은 현 시간에 추정된 상태에 대하여 얼마나 신뢰하는지 결정할 때 사용되며, 추정 상태의 신뢰도의 반비례한다.
- 상기 식에 사용되는 오차 공분산$P$의 경우 자이로센서와 가속도계에 의해 값이 결정되므로 2 x 2 행렬을 가지며, 이를 나타내면 다음과 같다.  
```math
P = \begin{bmatrix}
P_{00} & P{01} \\
P_{10} & P{11}
\end{bmatrix}
```  
#### 업데이트 단계
- 업데이트를 위하여 가장 먼저 측정치($z_k$)와 실제 상태($\hat {x}_{k \mid k-1}$)추정치와의 잔차를 구하여하하고, 이에 대한 식은 다음과 같다. 
  - $\tilde{y}_k = z_k + H_k \hat{x}_{k \mid k-1}$
- 여기서 관찰 모델 $H$는 이전 상태 추정치($\hat{x}_{k \mid k-1}$)를 관찰 공간으로 맵핑하기 위해 사용된다. 또한, 관찰 공간은 가속도계로부터 취득되기 때문에 잔차는 행렬이 아니다 
  - $\tilde{y}_k = \begin{bmatrix} \tilde {y} \end{bmatrix} _k$
- 추정치 잔차에 대한 공분산은 추정 공분산 $\hat{P}_{k \mid k-1}$과 관측 노이즈 $R$로부터 취득할 수 있으며 식은 다음과 같다.
  - $S_k = H_k \hat{P}_{k \mid k-1} H_k^T + R_k$
- 상기 식은 선험적으로 취득한 추정 공분산에 대하여 측정값을 얼마나 신뢰하는지 예측하는 것이며, 관측된 공간으로부터의 노이즈가 클수록 $S$는 커진다. 앞서 언급하였듯, 관찰 공간의 특성으로 인해 $S$는 행렬이 아니다.
  - $S_k = \begin{bmatrix} S\end{bmatrix}_k$
- 앞서 구한 식등을 통해 최적 칼만 이득을 구할 수 있으며, 이는 추정 공분산의 신뢰도에 따라 변화한다. 
- 이 때, 칼만 이득은 추정 공분산의 신뢰도에 반비례하며, 노이즈 잔차의 신뢰도에 비례하여 값이 선정된다.
  - $K_k = P_{k \mid k-1}H^T S_k^{-1}$
- 최초에 우리가 상태에 대하여 알 수 없을 경우 하기 값으로써 추정 공분산을 선정해야한다. (하기 식에 나타낸 $L$은 매우 큰 값을 의미)  
```math
P = \begin{bmatrix}
L & 0 \\
0 & L
\end{bmatrix}
```  
- 그러나, 짐벌 로봇의 경우 시작 각도를 알고 보정을 통해 실제 상태를 알 수 있으므로,  
```math
P = \begin{bmatrix}
0 & 0 \\
0 & 0
\end{bmatrix}
```  
- 이에 따라 사후 오차 공분산 행렬을 업데이트할 수 있으며, 이를 나타내는 식은 다음과 같다. ($I$는 2 x 2단위행렬을 의미)

### **칼만 필터 구현**
#### 예측 단계
- 칼만 필터를 구현하기 위해 먼저, 예측 단계가 진행되어야 한다. 이때 계산되어야 할 방정식은 총 2개이다.
- 해당 단계에서는 상태 추정치와 추정 공분산이 계산된다. 이를 코드로 구현하기 위해 다음과 같이 방정식을 정리하였다.  

**상태 추정치 방정식의 단순화 과정**  
```math
\hat{x}_{k \mid k-1} = F \hat {x}_{k-1 \mid k-1} + B \dot {\theta}_k
```  
```math
\begin{bmatrix}\theta \\ \dot {\theta}_b \end{bmatrix} =
\begin{bmatrix} 1 & -\triangle t \\ 0 & 1 \end{bmatrix}
\begin{bmatrix} \theta \\ \dot {\theta}_b \end{bmatrix}_{k-1 \mid k-1} +
\begin{bmatrix} \triangle t \\ 0 \end{bmatrix} \dot {\theta}_k
```  
```math
= \begin{bmatrix} \theta - \dot {\theta}_b \triangle t \\ \dot {\theta}_b \end{bmatrix}_{k-1 \mid k-1} + 
\begin{bmatrix} \triangle t \\ 0 \end{bmatrix} \dot {\theta}_k
```  
- 상기 식에 따라 각도의 추정치는 이전 상태의 추정치 $\theta_{k-1 \mid k-1}$에 현 시간의 각속도와 편향의 차 $\dot{\theta}_{k}-\dot{\theta}_{b}$에 시간 $\triangle t$ 만큼을 곱함으로 구할 수 있음을 알 수 있다.
- 이때, 편향은 일정함을 알 수 있다.
- 이를 나타낸 C++ 코드는 다음과 같다.
```c++
rate = newRate - bias; // dot{theta}_{k-1|k-1} - dot{theta}_b
angle += dt * rate; // theta_{k|k-1} = theta_{k-1|k-1} + rate * dt
```  

**공분산 추정치 방정식의 단순화 과정**  
```math
\hat {P}_{k \mid k-1} = F P_{k-1 \mid k-1}F_k^T + Q_k
```
```math
P = \begin{bmatrix} P_{00} & P{01} \\ P_{10} & P{11} \end{bmatrix} = \begin{bmatrix} 1 & -\triangle t \\ 0 & 1 \end{bmatrix} \begin{bmatrix} P_{00} & P_{01} \\ P_{10} & P_{11}\end{bmatrix}_{k-1 \mid k-1} \begin{bmatrix} 1 & 0 \\ - \triangle t & 1\end{bmatrix} + \begin{bmatrix} Q_{\theta} & 0 \\ 0 & Q_{\dot{\theta}_{b}}\end{bmatrix}\triangle t
```  
```math
= \begin{bmatrix} P_{00} + \triangle t (\triangle t P_{11} - P_{01} - P_{10} + Q_{\theta}) & P_{01} - \triangle t P_{11} \\ P_{10} - \triangle t P_{11} & P_{11} + Q_{\dot{\theta}_{b}} \triangle t\end{bmatrix}
```  

- 이를 C++ 코드로 표현하면 다음과 같다.  
```C++
P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
P[0][1] -= dt * P[1][1];
P[1][0] -= dt * P[1][1];
P[1][1] += Q_bias * dt;
```

### 업데이트 단계
- 업데이트를 위한 방정식 중 총 측정 후 잔차를 제외한 5단계의 과정이 요구된다.  
**추정치 잔차 방정식의 단순화 과정**  
```math
\tilde{y}_{k} = z_{k} - H \hat{x}_{k \mid k-1}
```  
```math
= z_{k} - \begin{bmatrix} 1 & 0 \end{bmatrix} \begin{bmatrix} \theta \\ \dot{\theta}_{k \mid k-1}\end{bmatrix}
```  
```math
= z_{k} - \theta_{k \mid k-1}
```  
- 이를 C++ 코드로 나타내면 다음과 같다.  
```C++
y = newAngle - angle;
```  

**추정치 잔차의 공분산 방정식 단순화 과정**  
```math
S_k = H_k \hat{P}_{k \mid k-1} H_k^T + R_k
```  
```math
= \begin{bmatrix} 1 & 0 \end{bmatrix} \begin{bmatrix} P_{00} & P_{01} \\ P_{10} & P_{11} \end{bmatrix}_{k \mid k-1} \begin{bmatrix} 1 \\ 0\end{bmatrix} + R
```  
```
= P_{00, k \mid k-1} + R
```  
```math
= P_{00, k \mid k-1} + var(v)
```  

- 이를 C++ 코드로 나타내면 다음과 같다.  
```C++
S = P[0][0] + R_measure;
```

**최적 칼만 이득 방정식 단순화 과정**  
```math
K_k = P_{k \mid k-1}H^T S_k^{-1}
```  

```math
\begin{bmatrix} K_{0} \\ K_{1} \end{bmatrix}_{k} = \begin{bmatrix} P_{00} & P_{01} \\ P_{10} & P_{11} \end{bmatrix}_{k \mid k-1} \begin{bmatrix} 1 \\ 0 \end{bmatrix} S_{k}^{-1}
```  
```math
\begin{bmatrix}P_{00} \\ P_{10} \end{bmatrix}_{k \mid k-1} S_{k}^{-1}
```  

```math
\frac{\begin{bmatrix} P_{00} \\ P_{10} \end{bmatrix}_{k \mid k-1}}{S_{k}}
```  
- 이를 C++ 코드로 나타내면 다음과 같다.  
```C++
K[0] = P[0][0] / S;
K[1] = P[1][0] / S;
```

**사후 상태 추정치 방정식 단순화 과정**  
```math
\hat{x}_{k \mid k} = \hat{x}_{k \mid k} + K_{k} \tilde{y}_{k}
```  
```math
\begin{bmatrix}\theta \\ \dot{\theta}_{b}\end{bmatrix}_{k \mid k} = \begin{bmatrix} \theta \\ \dot{\theta}_{b}\end{bmatrix}_{k \mid k-1} + \begin{bmatrix}K_{0} \\ K_{1}\end{bmatrix}_{k} \tilde{y}_{k}
```  
```math
= \begin{bmatrix}\theta \\ \dot{\theta}\end{bmatrix}_{k \mid k-1}+\begin{bmatrix}K_{0} \tilde{y} \\ K_{1} \tilde{y}\end{bmatrix}_{k}
```  

- 이를 C++ 코드로 나타내면 다음과 같다.
```C++
angle += K[0] * y;
bias  += K[1] * y;
```  

**사후  추정 공분산 방정식 단순화 과정**  
```math
P_{k \mid k} = (I - K_k H_k) \hat{P}_{k \mid k-1}
```  
```math
\begin{bmatrix}P_{00} & P_{01} \\ P_{10} & P_{11}\end{bmatrix}_{k \mid k}
= \left(\begin{bmatrix}1 & 0 \\ 0 & 1 \end{bmatrix} - \begin{bmatrix} K_{0} \\ K_{1} \end{bmatrix}_{k} \begin{bmatrix}1 & 0\end{bmatrix} \right) \begin{bmatrix}P_{00} & P_{01} \\ P_{10} & P_{11}\end{bmatrix}_{k \mid k-1}
```  
```math   
 = \left(\begin{bmatrix}1 & 0 \\ 0 & 1\end{bmatrix} - \begin{bmatrix}K_{0} & 0 \\ K_{1} & 0\end{bmatrix}_{k} \right) \begin{bmatrix}P_{00} & P_{01} \\ P_{10} & P_{11}\end{bmatrix}_{k \mid k-1}
```  
```math
=\begin{bmatrix}P_{00} & P_{01} \\ P_{10} & P_{11}\end{bmatrix}_{k \mid k-1} - \begin{bmatrix} K_{0} P_{00} & K_{0} P_{01} \\ K_{1} P_{00} & K_{1} P_{01}\end{bmatrix}
```   
 
 이를 C++ 코드로 나타내면 다음과 같다.  
```C++
float P00 = P[0][0];
float P01 = P[0][1];
P[0][0] -= K[0] * P00;
P[0][1] -= K[0] * P01;
P[1][0] -= K[1] * P00;
P[1][1] -= K[1] * P01;
```

- 저자에 따르면, 초기 지정값은 다음과 같을 때 최적 값을 가진다고 한다.
```C++
float Q_angle = 0.001;  
float Q_gyroBias = 0.003;  
float R_measure = 0.03;
```
> ### 참고자료
> 1. https://en.wikipedia.org/wiki/Kalman_filter
> 2. https://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/#comment-57783
> 3. https://blog.naver.com/heennavi1004/10183064672