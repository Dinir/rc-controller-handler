# 스크립트 역할

## ControlHandler

### RCControllerHandler
- 인스턴스를 생성하면 여기에 6축 컨트롤러의 이름과 축의 이름을 받아 저장합니다.
- `Poll()`을 실행할 때마다 저장한 컨트롤러의 축의 값을 업데이트하고, 변화가 있는 축의 번호를 기억합니다.
- `SendMessages()`를 실행해 원하는 게임오브젝트에 변화가 일어난 축에 대응하는 메세지를 보낼 수 있습니다.

#### FsSm600Handler : RCControllerHandler
- 6축 컨트롤러 핸들러를 상속 받아, FS-SM600 의 이름과 축의 이름을 넣어 인스턴스를 생성합니다.

### SextupleAxesManager
- 6축의 데이터를 여러가지 형태로 저장하고 불러올 수 있는 클래스입니다.
- 다음의 형태로 불러올 수 있습니다:
  - Vector2 `.Left`, Vector2 `.Right`, float `Aux`, float `Trigger`
  - `[0]` ~ `[5]`
- 각 축의 최대값을 설정해 입력 받는 값을 최대 값(1f)으로 처리할 수 있습니다.
- 마지막으로 업데이트하기 전의 축의 값을 불러와 변화가 있는 축의 번호를 받아올 수 있습니다.

### GeneralInputHandler
- 키보드&마우스, 게임패드의 입력을 SextupleAxesManager에서 관리하는 형태와 동일하게 가져옵니다.
- 어떤 키, 마우스 동작, 게임패드 버튼/스틱을 연결하는 지는 inputactions 에서 정의합니다.
  - 현재 프로젝트에서는 `Drone_Actions.inputactions` 에서 정의하고 있습니다.

## DroneMovement

### DroneController
- FsSm600Handler와 GeneralInputHandler의 인스턴스를 생성해 보관함으로써 양 측의 입력을 받아 관리합니다.
- FS-SM600이 연결되지 않았을 경우 일정 시간 간격으로 연결을 재시도합니다.
- FsSm600Handler의 `Poll()`을 주기적으로 실행하고 축의 변화를 감시합니다.
- 두 인스턴스는 DroneController 안에서 자신의 축에 변화가 생길 때마다 메세지를 보내고, DroneController는 이를 변화된 값과 함께 받아 다음의 메소드에 넣어 실행합니다:
  - `ActionLeft(Vector2)`, `ActionRight(Vector2)`, `ActionAux(float)`, `ActionTrigger(float)`
- 드론 날개의 애니메이션을 관리합니다.
  - HelperPlane 에 축 입력을 전달한 뒤 각 날개로부터 이 오브젝트까지의 거리를 가져와 날개의 회전수를 조정합니다. 

### HelperPlane

- 드론 모델의 위치에 보이지 않는 작은 평면을 두어, 입력을 받으면 기울어지게 합니다.
