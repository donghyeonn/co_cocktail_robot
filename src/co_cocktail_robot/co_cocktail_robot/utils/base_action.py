class BaseAction:
    def execute(self):
        raise NotImplementedError("execute()를 반드시 오버라이드 해야 합니다.")
    

# 동작별 액션을 모두 execute()라는 함수명으로 실행하기 위함.
# 여러 액션을 하나의 리스트에서 순서대로 실행할 수 있음.
# 기존 시스템에 유동성있는 추가 가능.