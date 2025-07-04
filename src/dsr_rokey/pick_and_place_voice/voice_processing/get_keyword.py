# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"

import os
import rclpy
import pyaudio
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain

from std_srvs.srv import Trigger
from voice_processing.MicController import MicController, MicConfig

from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT

############ Package Path & Environment Setting ############
current_dir = os.getcwd()
package_path = get_package_share_directory("pick_and_place_voice")

is_laod = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
openai_api_key = os.getenv("OPENAI_API_KEY")

############ AI Processor ############
# class AIProcessor:
#     def __init__(self):



############ GetKeyword Node ############
class GetKeyword(Node):
    def __init__(self):


        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.1, openai_api_key=openai_api_key
        )

        prompt_content = """
            당신은 바텐더를 보조하는 센스 넘치는 로봇입니다.
            사용자의 문장에서 특정 '물체'와 실행해야 할 '액션', '목적지'의 리스트를 추출해야 합니다.

            <목표>
            - 문장에서 물체 리스트에 포함된 물체를 정확히 추출하세요.
            - 문장에서 액션 리스트에 포함된 액션을 추출하세요.
            - 문장에서 목적지 리스트에 포함된 목적지를 추출하세요.
            - 특정 액션을 수행하기 위해 액션 리스트의 리스트를 조합하여 일련의 액션 과정을 만들어주세요.

            <물체 리스트>
            - green-juice, red-juice, blue-juice, tequila, lime, cherry, shaker

            <액션 리스트>
            - pick : 물건을 가져다 달라고 할 때, 맨 처음 진열장이나 가니쉬 위치에 있는 물건을 잡으러 가는 액션
            - take : 물건을 치워달라고 할 때, 맨 처음 pos1에 있는 물건을 잡으러 가는 액션
            - drop : 특정 행동을 수행하면서 특정 위치에 놓는 액션
            - pour : 특정 위치에 음료를 붓는 액션, 붓고 나서는 제자리에 놓는 것까지 수행하면 되므로 drop이 이어 나옴
            - open : 뚜껑을 여는 액션
            - close : 뚜껑을 닫는 액션
            - shake : 음료를 섞기 위해 셰이커를 흔드는 액션

            <목적지 리스트>
            - pos1 : 셰이커가 존재할, 뚜껑을 여닫게 될 위치
            - pos2 : 글라스 위치
            - pos3_1 : 음료 진열장에서의 tequila 위치
            - pos3_2 : 음료 진열장에서의 blue-juice 위치
            - pos3_3 : 음료 진열장에서의 green-juice 위치
            - pos3_4 : 음료 진열장에서의 red-juice 위치
            - pos4 : 바텐더가 작업하는 공간으로 물건을 주고 받는 위치. 무언가 가져다 놓아야 하는 상황에서 특정 목적지가 언급되지 않을 때 1순위로 사용될 목적지
            - pos5 : 바텐더가 작업하는 공간으로 물건을 주고 받는 위치. 무언가 여러 개 가져다 놓아야 하는 상황에서 특정 목적지가 언급되지 않고, pos4에 이미 물건을 가져다 놨을 때 2순위로 사용될 목적지

            <레시피>
            1. margarita
                - 기쁠 때 먹는 음료
                - 재료 : tequila, green-juice
            2. China Red
                - 화날 때 먹는 음료
                - 재료 : tequila, red-juice
            3. Blue Hawaii
                - 우울하거나 슬플 때 먹는 음료
                - 재료 : tequila, blue-juice
            4. Non Alcohol
                - 기분이 보통일 때 먹는 음료
                - 재료 : green-juice, blue-juice
                
            <출력 형식>
            - 추가적인 설명 없이 다음 형식에 맞춰서 출력해주세요: [[object_1, action_1, destination_1], [object_2, action_2, destination_2], ... ]
            - 액션을 단위로 각 리스트를 구성해주세요.
            - 수행할 액션이 대상이나 목적지가 필요 없이 수행해야 할 액션인 경우, 해당 값이 존재하지 않는다면 0으로 채워주세요.
            - 도구와 목적지의 순서는 등장 순서를 따릅니다.

            <특수 규칙>
            - 명확한 도구 명칭이 없지만 문맥상 유추 가능한 경우(예: "초록색 음료" → green-juice, "pos3_4에 있는 음료" → "red_juice")는 리스트 내 항목으로 최대한 추론해 반환하세요.
            - 다수의 도구와 목적지가 동시에 등장할 경우 각각에 대해 정확히 매칭하여 순서대로 출력하세요.
            - 문맥을 파악해서 실제로 필요한 것이 무엇인지 확인하여 반환해주세요.
            - 음료를 부어야 하는데 위치가 언급되지 않는다면 쉐이커에 부어주세요.

            <예시>
            - 입력: "데킬라를 pos5에 가져와줘"  
            출력: [['tequila', 'pick', 0], [0, 'drop', 'pos5']]

            - 입력: "데킬라 가져와줘" 
            출력: [['tequila', 'pick', 0], [0, 'drop', 'pos4']]

            - 입력: "그린 주스 치워줘"
            출력: [['green-juice', 'take', 0], [0, 'drop', 'pos3_3']]

            - 입력: "레드 주스 부어줘" 
            출력: [['red-juice', 'pick', 0], [0, 'pour', 'pos1'], [0, 'drop', 'pos3_4']]

            - 입력: "블루 주스 글라스에 부어줘" 
            출력: [['blue-juice', 'pick', 0], [0, 'pour', 'pos2'], [0, 'drop', 'pos3_2']]
            
            - 입력: "셰이커 좀 섞어줘"  
            출력: [[0, 'close', 0], ['tumbler', 'shake', 0], [0, 'open', 0], ['tumbler', 'take', 0], [0, 'drop', 'pos4']]

            <사용자 입력>
            "{user_input}"                
        """
        
        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)
        self.stt = STT(openai_api_key=openai_api_key)


        super().__init__("get_keyword_node")
        # 오디오 설정
        mic_config = MicConfig(
            chunk=12000,
            rate=48000,
            channels=1,
            record_seconds=5,
            fmt=pyaudio.paInt16,
            device_index=10,
            buffer_size=24000,
        )
        self.mic_controller = MicController(config=mic_config)
        # self.ai_processor = AIProcessor()

        self.get_logger().info("MicRecorderNode initialized.")
        self.get_logger().info("wait for client's request...")
        self.get_keyword_srv = self.create_service(
            Trigger, "get_keyword", self.get_keyword
        )
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

    def extract_keyword(self, output_message):
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response["text"]

        object, target = result.strip().split("/")

        object = object.split()
        target = target.split()

        print(f"llm's response: {object}")
        print(f"object: {object}")
        print(f"target: {target}")
        return object
    
    def get_keyword(self, request, response):  # 요청과 응답 객체를 받아야 함
        try:
            print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("please check your device index")
            return None

        while not self.wakeup_word.is_wakeup():
            pass

        # STT --> Keword Extract --> Embedding
        output_message = self.stt.speech2text()
        keyword = self.extract_keyword(output_message)

        self.get_logger().warn(f"Detected tools: {keyword}")

        # 응답 객체 설정
        response.success = True
        response.message = " ".join(keyword)  # 감지된 키워드를 응답 메시지로 반환
        return response


def main():
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
