# get_keyword_langgraph.py
import os
import rclpy
import pyaudio
import json
import pandas as pd
from rclpy.node import Node

from ament_index_python.packages import get_package_prefix
from dotenv import load_dotenv
from std_srvs.srv import Trigger
from cocktail_interfaces.srv import GetKeywordList
from cocktail_interfaces.msg import KeywordGroup

from langgraph.graph import StateGraph, END
from langchain.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain
from langchain.embeddings import OpenAIEmbeddings
from langchain.vectorstores import FAISS
from langchain.schema.runnable import RunnableConfig
from langchain.chains.combine_documents.stuff import StuffDocumentsChain
from langchain.chains.retrieval_qa.base import RetrievalQA
from langchain.text_splitter import CharacterTextSplitter
from langchain.schema.document import Document

from .MicController import MicController, MicConfig
from .wakeup_word import WakeupWord
from .stt import STT

# ========== 환경 설정 ==========
package_path = get_package_prefix("co_cocktail_robot")
load_dotenv(dotenv_path=os.path.join(f"{package_path}/share/co_cocktail_robot/resource/.env"))
openai_api_key = os.getenv("OPENAI_API_KEY")

# ========== 텍스트 파일 -> FAISS ==========
embedding = OpenAIEmbeddings(openai_api_key=openai_api_key)
text_splitter = CharacterTextSplitter(chunk_size=50, chunk_overlap=10)

def load_faiss_from_file(file_path, index_name):
    with open(file_path, "r") as f:
        content = f.read()
    docs = text_splitter.create_documents([content])
    return FAISS.from_documents(docs, embedding)

def load_faiss_from_json(file_path, index_name):
    with open(file_path, "r", encoding="utf-8") as f:
        raw_data = json.load(f)

    docs = []
    for item in raw_data:
        content = "\n".join([f"{k}: {v}" for k, v in item.items()])
        docs.append(Document(page_content=content))

    return FAISS.from_documents(docs, embedding)

recipe_db = load_faiss_from_json(os.path.join(package_path, "share/co_cocktail_robot/resource/recipe.json"), "recipe")
customer_db = load_faiss_from_json(os.path.join(package_path, "share/co_cocktail_robot/resource/customers.json"), "customers")


# ========== LangChain / LangGraph 설정 ==========
llm = ChatOpenAI(model="gpt-4.1", temperature=0.3, openai_api_key=openai_api_key)

OBJECTS = {"green-juice", "red-juice", "blue-juice", "tequila", "lime", "cherry", "shaker"}
ACTIONS = {"pick", "take", "drop", "pour", "open", "close", "shake"}
DESTINATIONS = {"pos1", "pos2", "pos3_1", "pos3_2", "pos3_3", "pos3_4", "pos4", "pos5"}

# Classifier Node
prompt_classify_route = PromptTemplate(
    input_variables=["user_input"],
    template="""
    아래 사용자 입력을 읽고 다음 중 하나로 분류하세요:

    - DB_UPDATE: 단골 정보나 레시피 정보를 업데이트해야 하는 요청입니다.
        <예시>
            - "마르가리타 음료 정보를 지워줘"
            - "정환의 단골 기록 지워줘"
            - "단골 동현이 마르가리타를 제일 좋아한다고 수정해줘"
            - "마르가리타 재료를 데킬라랑 라임으로 바꿔줘"

    - TASK: 로봇이 음료를 만들거나 가져오는 작업입니다. DB_UPDATE에 해당하지 않는 작업도 해당합니다.
        <예시>
            - "데킬라를 pos5에 가져와줘"  
            - "데킬라 가져와줘" 
            - "그린 주스 치워줘"
            - "레드 주스 부어줘" 
            - "블루 주스 글라스에 부어줘" 
            - "셰이커 좀 섞어줘"  
            - "데킬라랑 그린 주스 준비해줘"
            - "동현 씨가 자주 마시는 칵테일 재료 준비해줘"
            - "우울할 때 마시는 칵테일 재료 치우고 기분 좋을 때 마시는 칵테일 재료 준비해줘"

    그 외 응답 없이 반드시 위의 단어 중 하나만 출력하세요.

    사용자 입력: "{user_input}"
    """
)
classify_chain = LLMChain(llm=llm, prompt=prompt_classify_route)

def classify_route(state):
    user_input = state["user_input"]
    result = classify_chain.invoke({"user_input": user_input})
    input_type = result["text"].strip().upper()
    return {**state, "type": input_type}

# Agent 0: RAG 필요 여부 판단
prompt_agent0 = PromptTemplate(
    input_variables=["user_input"],
    template="""
    사용자의 요청을 읽고, 다음 중 어떤 외부 정보가 필요한지 판단하세요:
    - 칵테일의 재료나 조합을 알아야 하는 경우: 'RECIPE'
    - 최근 주문, 자주 마신 음료 등 손님 정보를 알아야 하는 경우: 'CUSTOMER'
    - 둘 다 필요한 경우: 'BOTH'
    - 외부 정보가 필요 없는 경우: 'NONE'

    반드시 위의 단어 중 하나로만 대답하세요.

    사용자 입력: {user_input}
    """
)
agent0_chain = LLMChain(llm=llm, prompt=prompt_agent0)

def agent0(state):
    query = state["user_input"]
    result = agent0_chain.invoke({"user_input": query})
    decision = result["text"].strip().upper()

    # 값 보정
    if decision not in {"RECIPE", "CUSTOMER", "BOTH"}:
        decision = "NONE"
    print(f"agent0 decision:{decision}")
    return {
        "user_input": query,
        "need_rag": decision != "NONE",
        "rag_type": decision
    }

# Agent 1: 태스크 생성
prompt_agent1 = PromptTemplate(
    input_variables=["user_input", "context"],
    template="""
    당신은 바텐더를 보조하는 센스 넘치는 로봇입니다.
    사용자의 문장에서 특정 '물체'와 실행해야 할 '액션', '목적지'의 리스트를 추출해야 합니다.
    추가 참조 사항이 주어질 경우, 해당 사항도 참조해주세요.

    <목표>
    - 문장에서 물체 리스트에 포함된 물체를 정확히 추출하세요.
    - 문장에서 액션 리스트에 포함된 액션을 추출하세요.
    - 문장에서 목적지 리스트에 포함된 목적지를 추출하세요.
    - 특정 액션을 수행하기 위해 액션 리스트의 리스트를 조합하여 일련의 액션 과정을 만들어주세요.

    <물체 리스트>
    - green-juice, red-juice, blue-juice, tequila, lime, cherry, shaker

    <액션 리스트>
    - pick : 물건을 가져다 달라고 할 때, 맨 처음 진열장이나 가니쉬 위치에 있는 물건이나, pos1에 있는 셰이커를 잡으러 가는 액션
    - take : 물건을 치워달라고 할 때, 맨 처음 pos4나 pos5에 있는 물건을 잡으러 가는 액션. 해당 액션은 목적지가 명시되면 안 됨.
    - drop : 특정 행동을 수행하면서 특정 위치에 놓는 액션
    - pour : 특정 위치에 음료를 붓는 액션, 붓고 나서는 제자리에 놓는 것까지 수행하면 되므로 drop이 이어 나옴
    - open : 뚜껑을 여는 액션
    - close : 뚜껑을 닫는 액션
    - shake : 음료를 섞기 위해 pos1에 있는 셰이커를 흔드는 액션

    <목적지 리스트>
    - pos1 : 셰이커가 존재할, 뚜껑을 여닫게 될 위치
    - pos2 : 글라스 위치
    - pos3_1 : 음료 진열장에서의 tequila 원래 위치
    - pos3_2 : 음료 진열장에서의 blue-juice 원래 위치
    - pos3_3 : 음료 진열장에서의 green-juice 원래 위치
    - pos3_4 : 음료 진열장에서의 red-juice 원래 위치
    - pos4 : 바텐더가 작업하는 공간으로 물건을 주고 받는 위치. 무언가 가져다 놓아야 하는 상황에서 특정 목적지가 언급되지 않을 때 1순위로 사용될 목적지
    - pos5 : 바텐더가 작업하는 공간으로 물건을 주고 받는 위치. 무언가 여러 개 가져다 놓아야 하는 상황에서 특정 목적지가 언급되지 않고, pos4에 이미 물건을 가져다 놨을 때 2순위로 사용될 목적지

    <출력 형식>
    - 추가적인 설명 없이 다음 형식에 맞춰서 출력해주세요: [[object_1, action_1, destination_1], [object_2, action_2, destination_2], ... ]
    - 액션을 단위로 각 리스트를 구성해주세요.
    - 수행할 액션이 대상이나 목적지가 필요 없이 수행해야 할 액션인 경우, 해당 값이 존재하지 않는다면 'X'으로 채워주세요.
    - 도구와 목적지의 순서는 등장 순서를 따릅니다.

    <특수 규칙>
    - 명확한 도구 명칭이 없지만 문맥상 유추 가능한 경우(예: "초록색 음료" → green-juice, "pos3_4에 있는 음료" → "red_juice")는 리스트 내 항목으로 최대한 추론해 반환하세요.
    - 다수의 도구와 목적지가 동시에 등장할 경우 각각에 대해 정확히 매칭하여 순서대로 출력하세요.
    - 문맥을 파악해서 실제로 필요한 것이 무엇인지 확인하여 반환해주세요.
    - 음료를 부어야 하는데 위치가 언급되지 않는다면 셰이커에 부어주세요.
    - 라임이나 체리와 같은 가니쉬를 놓아야 하는데 위치가 언급되지 않는다면 글래스에 놓아주세요.
    - 따옴표는 ‘(U+2018)를 사용하지 마시고 '를 사용해주세요.

    <예시>
    - 입력: "데킬라를 pos5에 가져와줘"  
    출력: [['tequila', 'pick', 'X'], ['X', 'drop', 'pos5']]

    - 입력: "데킬라 가져와줘" 
    출력: [['tequila', 'pick', 'X'], ['X', 'drop', 'pos4']]

    - 입력: "그린 주스 치워줘"
    출력: [['green-juice', 'take', 'X'], ['X', 'drop', 'pos3_3']]

    - 입력: "레드 주스 부어줘" 
    출력: [['red-juice', 'pick', 'X'], ['X', 'pour', 'pos1'], ['X', 'drop', 'pos3_4']]

    - 입력: "블루 주스 글라스에 부어줘" 
    출력: [['blue-juice', 'pick', 'X'], ['X', 'pour', 'pos2'], ['X', 'drop', 'pos3_2']]
    
    - 입력: "셰이커 좀 섞어줘"  
    출력: [['shaker', 'take', 'X'], ['X', 'drop', 'pos1'], ['X', 'close', 'X'], ['X', 'shake', 'X'] , ['X', 'open', 'X'], ['shaker', 'pick', 'X'], ['X', 'drop', 'pos4']

    - 입력: "데킬라랑 그린 주스 준비해줘"
    출력: [['tequila', 'pick', 'X'], ['X', 'drop', 'pos4'], ['green-juice', 'pick', 'X'], ['X', 'drop', 'pos5']]

    <사용자 입력>
    "{user_input}" 
    <추가 참조 사항>
    "{context}"
    """
)
agent1_chain = LLMChain(llm=llm, prompt=prompt_agent1)

def agent1(state):
    query = state["user_input"]
    rag_type = state.get("rag_type", "NONE")
    retry_count = state.get("retry_count", 0)
    context = ""

    if rag_type in {"RECIPE", "BOTH"}:
        recipe_result = recipe_db.similarity_search(query)
        context += "\n".join([d.page_content for d in recipe_result])

    if rag_type in {"CUSTOMER", "BOTH"}:
        cust_result = customer_db.similarity_search(query)
        context += "\n" + "\n".join([d.page_content for d in cust_result])

    result = agent1_chain.invoke({"user_input": query, "context": context})
    print(f"result:{result}")
    return {
        "task_list": result["text"],
        "user_input": query,
        "need_rag": state.get("need_rag", False),
        "rag_type": rag_type,
        "retry_count": retry_count
    }


# Agent 2: 검증 (LLM으로 대체)
prompt_agent2 = PromptTemplate(
    input_variables=["task_list", "objects", "actions", "destinations"],
    template="""
    당신은 로봇 태스크 플래닝 에이전트의 결과값을 검사하는 에이전트입니다.
    플래닝 에이전트는 특정 조건을 따라 결과값을 도출합니다.
    검사 규칙에 따라 task_list가 유효한지 판단하세요.

    
    <플래닝 에이전트가 따르는 조건>
        당신은 바텐더를 보조하는 센스 넘치는 로봇입니다.
        사용자의 문장에서 특정 '물체'와 실행해야 할 '액션', '목적지'의 리스트를 추출해야 합니다.
        추가 참조 사항이 주어질 경우, 해당 사항도 참조해주세요.

        1.목표
        - 문장에서 물체 리스트에 포함된 물체를 정확히 추출하세요.
        - 문장에서 액션 리스트에 포함된 액션을 추출하세요.
        - 문장에서 목적지 리스트에 포함된 목적지를 추출하세요.
        - 특정 액션을 수행하기 위해 액션 리스트의 리스트를 조합하여 일련의 액션 과정을 만들어주세요.

        2.물체 리스트
        - green-juice, red-juice, blue-juice, tequila, lime, cherry, shaker

        3.액션 리스트
        - pick : 물건을 가져다 달라고 할 때, 맨 처음 진열장이나 가니쉬 위치에 있는 물건이나, pos1에 있는 셰이커를 잡으러 가는 액션
        - take : 물건을 치워달라고 할 때, 맨 처음 pos4나 pos5에 있는 물건을 잡으러 가는 액션. 해당 액션은 목적지가 명시되면 안 됨.
        - drop : 특정 행동을 수행하면서 특정 위치에 놓는 액션
        - pour : 특정 위치에 음료를 붓는 액션, 붓고 나서는 제자리에 놓는 것까지 수행하면 되므로 drop이 이어 나옴
        - open : 뚜껑을 여는 액션
        - close : 뚜껑을 닫는 액션
        - shake : 음료를 섞기 위해 pos1에 있는 셰이커를 흔드는 액션

        4.목적지 리스트
        - pos1 : 셰이커가 존재할, 뚜껑을 여닫게 될 위치
        - pos2 : 글라스 위치
        - pos3_1 : 음료 진열장에서의 tequila 원래 위치
        - pos3_2 : 음료 진열장에서의 blue-juice 원래 위치
        - pos3_3 : 음료 진열장에서의 green-juice 원래 위치
        - pos3_4 : 음료 진열장에서의 red-juice 원래 위치
        - pos4 : 바텐더가 작업하는 공간으로 물건을 주고 받는 위치. 무언가 가져다 놓아야 하는 상황에서 특정 목적지가 언급되지 않을 때 1순위로 사용될 목적지
        - pos5 : 바텐더가 작업하는 공간으로 물건을 주고 받는 위치. 무언가 여러 개 가져다 놓아야 하는 상황에서 특정 목적지가 언급되지 않고, pos4에 이미 물건을 가져다 놨을 때 2순위로 사용될 목적지

        5.출력 형식
        - 추가적인 설명 없이 다음 형식에 맞춰서 출력해주세요: [[object_1, action_1, destination_1], [object_2, action_2, destination_2], ... ]
        - 액션을 단위로 각 리스트를 구성해주세요.
        - 수행할 액션이 대상이나 목적지가 필요 없이 수행해야 할 액션인 경우, 해당 값이 존재하지 않는다면 'X'으로 채워주세요.
        - 도구와 목적지의 순서는 등장 순서를 따릅니다.

        6.특수 규칙
        - 명확한 도구 명칭이 없지만 문맥상 유추 가능한 경우(예: "초록색 음료" → green-juice, "pos3_4에 있는 음료" → "red_juice")는 리스트 내 항목으로 최대한 추론해 반환하세요.
        - 다수의 도구와 목적지가 동시에 등장할 경우 각각에 대해 정확히 매칭하여 순서대로 출력하세요.
        - 문맥을 파악해서 실제로 필요한 것이 무엇인지 확인하여 반환해주세요.
        - 음료를 부어야 하는데 위치가 언급되지 않는다면 셰이커에 부어주세요.
        - 라임이나 체리와 같은 가니쉬를 놓아야 하는데 위치가 언급되지 않는다면 글래스에 놓아주세요.
        - 따옴표는 ‘(U+2018)를 사용하지 마시고 '를 사용해주세요.

        7.예시
        - 입력: "데킬라를 pos5에 가져와줘"  
        출력: [['tequila', 'pick', 'X'], ['X', 'drop', 'pos5']]

        - 입력: "데킬라 가져와줘" 
        출력: [['tequila', 'pick', 'X'], ['X', 'drop', 'pos4']]

        - 입력: "그린 주스 치워줘"
        출력: [['green-juice', 'take', 'X'], ['X', 'drop', 'pos3_3']]

        - 입력: "레드 주스 부어줘" 
        출력: [['red-juice', 'pick', 'X'], ['X', 'pour', 'pos1'], ['X', 'drop', 'pos3_4']]

        - 입력: "블루 주스 글라스에 부어줘" 
        출력: [['blue-juice', 'pick', 'X'], ['X', 'pour', 'pos2'], ['X', 'drop', 'pos3_2']]

        - 입력: "데킬라 글라스에 부어줘" 
        출력: [['tequila', 'pick', 'X'], ['X', 'pour', 'pos2'], ['X', 'drop', 'pos3_1']]
        
        - 입력: "셰이커 좀 섞어줘"  
        출력: [['shaker', 'take', 'X'], ['X', 'drop', 'pos1'], ['X', 'close', 'X'], ['X', 'shake', 'X'] , ['X', 'open', 'X'], ['shaker', 'pick', 'X'], ['X', 'drop', 'pos4']

        - 입력: "데킬라랑 그린 주스 준비해줘"
        출력: [['tequila', 'pick', 'X'], ['X', 'drop', 'pos4'], ['green-juice', 'pick', 'X'], ['X', 'drop', 'pos5']]


    <검사 규칙>
    1. task_list는 태스크 플래닝 에이전트가 입력을 받고 일련의 움직임을 담은 것입니다.        
    2. 리스트 내 각 항목은 object, action, destination 순의 3요소 리스트여야 합니다.
    3. 각 요소는 다음 중 하나여야 합니다:
        - object: {objects}
        - action: {actions}
        - destination: {destinations}
        - 또는 'X'로 표시된 무시 가능한 항목
    4. 문맥상 논리적이어야 합니다:
        - pick/take로 무언가를 잡지 않고 pour/drop 수행 불가
        - shake 전에 close가 반드시 있어야 함
        - task_list가 빈 리스트면 안 됨
    5. 목적지 리스트는 해당 물건들의 항상 있어야 하는 위치가 아닙니다. 플래닝 에이전트는 바텐더와 협업하는 로봇이기 때문에 물건을 특정 위치에서 주고 받을 수 있기 때문입니다.

    task_list: {task_list}
    위 조건을 모두 만족하면 'VALID'만 출력하시고, 하나라도 위반하면 'INVALID'와 이유를 출력하세요.
    """
)
agent2_chain = LLMChain(llm=llm, prompt=prompt_agent2)

def agent2(state):
    print(f'agent2 state:{state}')
    task_list = state["task_list"]
    result = agent2_chain.invoke({
        "task_list": task_list,
        "objects": ", ".join(OBJECTS),
        "actions": ", ".join(ACTIONS),
        "destinations": ", ".join(DESTINATIONS),
    })
    print(f'agent2 result:{result}')
    decision = result["text"].strip().upper()
    valid = decision == "VALID"
    retry_count = state.get("retry_count", 0)

    if not valid and retry_count >= 2:
        # 재시도 2회 초과 시 fallback
        return {
            "user_input": state["user_input"],
            "task_list": "[['X', 'head_tilt', 'X']]",
            "need_rag": state.get("need_rag", False),
            "valid": True,
            "retry_count": retry_count
        }

    return {
        "user_input": state["user_input"],
        "task_list": task_list,
        "need_rag": state.get("need_rag", False),
        "valid": valid,
        "retry_count": retry_count + 1 if not valid else retry_count
    }

# Agent 3 : DB update
update_prompt = PromptTemplate(
    input_variables=["user_input"],
    template="""
    당신은 바텐더 로봇의 DB 업데이트를 담당하는 에이전트입니다.
    사용자의 입력을 읽고 다음 형식으로 필요한 정보를 JSON으로 정확히 추출하세요.
    이름을 제외한 값들은 영어로 작성해주세요.
    이름의 경우 성이나 호칭은 제외하고 이름만 작성해주세요.

    <출력 형식>
    {{
      "action": "UPDATE" 또는 "DELETE",
      "table": "customers" 또는 "recipe",
      "key": "수정 또는 삭제할 항목의 키 (예: 손님 이름 또는 칵테일 이름)",
      "field": "수정할 필드명 (삭제할 경우 생략 가능)",
      "value": "새로운 값 (삭제할 경우 생략 가능)"
    }}

    <예시>
    - "마르가리타 음료 정보를 지워줘"
    → {{
      "action": "DELETE",
      "table": "recipe",
      "key": "Margarita"
    }}

    - "정환의 단골 기록 지워줘"
    → {{
      "action": "DELETE",
      "table": "customers",
      "key": "정환"
    }}

    - "단골 동현이 마르가리타를 제일 좋아한다고 수정해줘"
    → {{
      "action": "UPDATE",
      "table": "customers",
      "key": "동현",
      "field": "favorite",
      "value": "Margarita"
    }}

    - "마르가리타 재료를 데킬라랑 라임으로 바꿔줘"
    → {{
      "action": "UPDATE",
      "table": "recipe",
      "key": "Margarita",
      "field": "ingredients",
      "value": "tequila, lime"
    }}

    <주의 사항>
    - 반드시 위 JSON 형식 그대로 출력하세요.
    - 다른 말은 하지 마세요. JSON 외의 텍스트가 포함되면 안 됩니다.
    - 삭제 시에는 field와 value 생략 가능
    사용자 입력: "{user_input}"
    """
)

def update_database(update_info, base_path):
    action = update_info.get("action", "UPDATE").upper()
    table = update_info.get("table")
    key = update_info.get("key")
    field = update_info.get("field", "")
    value = update_info.get("value", "")

    # 경로 설정
    if table == "customers":
        path = os.path.join(base_path, "customers.json")
    elif table == "recipe":
        path = os.path.join(base_path, "recipe.json")
    else:
        raise ValueError(f"Unknown table name: {table}")

    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except FileNotFoundError:
        data = []

    if action == "DELETE":
        original_len = len(data)
        data = [item for item in data if item.get("name") != key]
        if len(data) < original_len:
            print(f"[DB 삭제 완료] {table}: {key} 항목 삭제됨")
        else:
            print(f"[DB 삭제 실패] {table}: {key} 항목 없음")
    else:
        updated = False
        for item in data:
            if item.get("name") == key:
                item[field] = value
                updated = True
                break

        if not updated:
            if table == "customers":
                new_item = {
                    "name": key,
                    "latest_order": value if field == "latest_order" else "",
                    "favorite": value if field == "favorite" else ""
                }
            elif table == "recipe":
                new_item = {
                    "name": key,
                    "description": value if field == "description" else "",
                    "ingredients": value if field == "ingredients" else []
                }
            data.append(new_item)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

    print(f"[DB 업데이트 완료] {table}: {key}의 {field} → {value}")

update_chain = LLMChain(llm=llm, prompt=update_prompt)

def agent3(state):
    user_input = state["user_input"]
    print(f"[agent3] 입력: {user_input}")

    try:
        result = update_chain.invoke({"user_input": user_input})
        print(f"[agent3] LLM 출력: {result}")

        update_info = json.loads(result["text"])

        # base_path 지정 (리소스 폴더 기준)
        base_path = os.path.join(package_path, "share/co_cocktail_robot/resource")

        # DB 업데이트 실행
        update_database(update_info, base_path)

        return {
            "user_input": user_input,
            "task_list": "[['X', 'head_nod', 'X']]",  # 긍정 응답
            "need_rag": False,
            "valid": True,
            "retry_count": 0,
            "type": "UPDATE"
        }

    except Exception as e:
        print(f"[agent3] 오류: {e}")
        return {
            "user_input": user_input,
            "task_list": "[['X', 'head_tilt', 'X']]",  # 부정 응답
            "need_rag": False,
            "valid": True,
            "retry_count": 0,
            "type": "UPDATE"
        }
    
    
from typing import TypedDict
class GraphState(TypedDict):
    user_input: str
    task_list: str
    need_rag: bool
    rag_type: str
    valid: bool
    retry_count: int 
    task_type: str

# LangGraph 구성
workflow = StateGraph(GraphState)

workflow.add_node("classify_route", classify_route)
workflow.add_node("agent0", agent0)
workflow.add_node("agent1", agent1)
workflow.add_node("agent2", agent2)
workflow.add_node("agent3", agent3)

workflow.set_entry_point("classify_route")

workflow.add_conditional_edges("classify_route", lambda x: x["type"], {
    "DB_UPDATE": "agent3",
    "TASK": "agent0"
})

workflow.add_edge("agent0", "agent1")
workflow.add_edge("agent1", "agent2")
workflow.add_conditional_edges("agent2", lambda x: "agent1" if not x.get("valid") else END)
workflow.add_edge("agent3", END)

langgraph_executor = workflow.compile()

# ========== ROS Node ==========
class GetKeyword(Node):
    def __init__(self):
        super().__init__("get_keyword_node")
        self.stt = STT(openai_api_key=openai_api_key)
        mic_config = MicConfig(chunk=12000, rate=48000, channels=1, record_seconds=5, fmt=pyaudio.paInt16, device_index=12, buffer_size=24000)
        self.mic_controller = MicController(config=mic_config)
        self.wakeup_word = WakeupWord(mic_config.buffer_size)
        self.get_keyword_srv = self.create_service(GetKeywordList, "/get_keyword", self.get_keyword)
        self.get_logger().info("Ready for keyword extraction with LangGraph.")

    def get_keyword(self, request, response):
        try:
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
            self.get_logger().info("Wake up is set.")
        except Exception as e:
            self.get_logger().error(e)#"Audio stream error. Check device.")
            return None

        while not self.wakeup_word.is_wakeup():
            print(self.wakeup_word.is_wakeup())
            pass

        user_input = self.stt.speech2text()
        result = langgraph_executor.invoke({"user_input": user_input})

        self.get_logger().info(f"Final Task List: {result['task_list']}")
        response.keywords = []
        for group in eval(result["task_list"]):
            kg = KeywordGroup()
            kg.items = group
            response.keywords.append(kg)
        return response


def main():
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
