"""지원 공구 6종의 alias와 오인식 후보 데이터.

여기 있는 문자열은 로직이 아니라 현장 발화/Whisper 오인식 데이터를 담은 표다.
새 공구를 추가하거나 별칭을 늘릴 때는 `TOOL_IDS`, `DISPLAY_NAMES`, `ALIASES`,
필요 시 `NUMBER_HINTS`와 `UNCERTAIN_ALIASES`를 함께 갱신해야 한다.
"""

from __future__ import annotations

# 현재 DB toolbox layout에서 지원하는 tool_id 목록.
TOOL_IDS = (
    "screwdriver",
    "utility_knife",
    "ratchet_wrench",
    "multi_tool",
    "spanner_16mm",
    "socket_19mm",
)

UNKNOWN_TOOL_ID = "unknown"

# 사용자 확인 문구와 로그에 표시할 사람이 읽기 쉬운 이름.
DISPLAY_NAMES = {
    "screwdriver": "설치 드라이버",
    "utility_knife": "커터 칼",
    "ratchet_wrench": "라쳇 렌치",
    "multi_tool": "멀티툴",
    "spanner_16mm": "스패너 16mm",
    "socket_19mm": "복스 소켓 19mm",
    "unknown": "알 수 없음",
}

# 공구별 별칭 목록.
# 같은 공구를 부르는 한국어 표현, 영어 표현, Whisper가 자주 만들 수 있는
# 비슷한 발음의 오인식 표현을 함께 둔다.
ALIASES = {
    "screwdriver": [
        "드라이버", "드라이브", "드라이버를", "드라이버로", "드라이버가", "드라이버야",
        "드라이버 줘", "드라이버 주세요", "드라이버 찾아줘", "드라이버 가져와", "드라이버 갖고 와",
        "스크류드라이버", "스크루드라이버", "스크류 드라이버", "스크루 드라이버",
        "스크류 다이버", "스크루 다이버", "스쿠루 드라이버", "스쿠류 드라이버",
        "스크류 드라이브", "스크루 드라이브",
        "driver", "drivers", "drive", "driving",
        "screwdriver", "screw driver", "screwdrivers", "screw drive",
        "screw", "screws", "bit driver",
        "비트 드라이버", "전동 드라이버", "임팩 드라이버", "임팩트 드라이버",
        "십자 드라이버", "일자 드라이버",
        "설치 드라이버", "프린터 드라이버", "그래픽 드라이버", "장치 드라이버", "운전기사",
    ],
    "utility_knife": [
        "커터칼", "커터 칼", "컷터칼", "컷터 칼",
        "커터", "컷터", "카터",
        "커터를", "커터로", "커터가", "커터야",
        "커터 줘", "커터칼 줘", "커터칼 주세요", "커터칼 가져와", "커터칼 찾아줘",
        "칼", "칼 줘", "칼 가져와",
        "나이프", "나이프 줘",
        "유틸리티 나이프", "유틸리티 라이프", "유틸리티 나이브",
        "유틸리티 knife", "utility knife", "utility life", "utility live",
        "utility knives", "utility night",
        "유틸 나이프", "유틸 칼",
        "박스커터", "박스 커터", "box cutter",
        "복스 커터", "박스 칼",
        "커팅 나이프", "cutting knife", "cutter knife", "cutter",
        "cut her", "cutter call",
        "카터칼", "카터 칼",
        "아트나이프", "아트 나이프", "art knife",
        "안전칼", "작업칼", "공업용 칼", "칼날", "커터날",
    ],
    "ratchet_wrench": [
        "라쳇 렌치", "라쳇렌치",
        "라체트 렌치", "라체트렌치",
        "라챗 렌치", "라챗렌치",
        "라칫 렌치", "라칫렌치",
        "래칫 렌치", "래칫렌치",
        "래쳇 렌치", "래쳇렌치",
        "라쳇", "라체트", "라챗", "라칫", "래칫", "래쳇",
        "라켓", "라켓 렌치", "로켓 렌치", "라켓 런치", "로켓 런치",
        "ratchet wrench", "rachet wrench", "ratchet ranch", "ratchet lunch",
        "ratchet launch", "ratchet winch", "racket wrench",
        "rocket wrench", "rocket launch",
        "wrench", "렌치", "렌찌", "랜치", "런치", "런지", "렌즈", "랜즈",
        "렌치 줘", "라쳇 줘", "라쳇렌치 줘", "라쳇렌치 가져와",
        "소켓 렌치", "복스 렌치", "토크 렌치", "몽키 렌치",
        "스패너 렌치", "라쳇 핸들", "복스 핸들", "라쳇 드라이버",
    ],
    "multi_tool": [
        "멀티툴", "멀티 툴",
        "멀티툴 줘", "멀티툴 가져와", "멀티툴 찾아줘",
        "멀티 도구", "멀티 공구",
        "다용도 공구", "다기능 공구", "다목적 공구",
        "만능 공구", "만능툴",
        "multi tool", "multitool", "multi-tool",
        "multi tools", "multi two", "multi to", "multi too",
        "multi tu", "multi tour",
        "multi tool please",
        "tool", "tools", "툴", "투울", "툴 줘",
        "공구", "공구 줘", "공구 가져와", "공구 찾아줘",
        "툴킷", "toolkit", "tool kit",
        "공구세트", "공구 세트", "멀티 키트",
        "맥가이버 칼", "맥가이버 나이프",
        "swiss army knife", "스위스 아미 나이프",
        "플라이어", "플라이어 툴", "pliers", "player",
        "멀티탭", "멀티탭 줘", "멀티미터", "multimeter", "멀티미디어", "멀티플레이",
    ],
    "spanner_16mm": [
        "스패너", "스패너 줘", "스패너 주세요", "스패너 가져와", "스패너 찾아줘",
        "스패너 십육미리", "스패너 십육 밀리", "스패너 16미리", "스패너 16 밀리",
        "16미리 스패너", "십육미리 스패너", "스패너 열여섯 미리", "열여섯 미리 스패너",
        "스파너", "스페너", "스페나", "스패나",
        "스페인어", "스페인", "스페니어", "스페너가", "스패너가",
        "스페인어 줘", "스페인어로", "스페인어를",
        "spanish", "Spanish", "Spanish language", "스페니쉬", "스페니시", "스페인어 번역",
        "spanner", "spanner 16", "spanner sixteen",
        "spanner sixteen millimeter", "spanner sixteen millimeters",
        "spanner 16 millimeter", "spanner 16 mm",
        "spaner", "spanner wrench",
        "scanner", "scanner 16", "스캐너", "스캐너 줘", "스캐너 16",
        "스패너 십구미리", "스패너 십칠미리", "스패너 십오미리", "스패너 육미리",
        "스패너 6미리", "스패너 60미리", "스패너 15미리", "스패너 17미리", "스패너 19미리",
        "렌치 16미리", "16미리 렌치",
        "스패너 렌치", "오픈 렌치", "양구 렌치", "양구 스패너",
        "콤비네이션 렌치", "combination wrench", "open end wrench",
        "sixteen millimeter wrench", "sixteen mm wrench",
    ],
    "socket_19mm": [
        "소켓", "소켓 줘", "소켓 주세요", "소켓 가져와", "소켓 찾아줘",
        "소켓 19미리", "소켓 십구미리", "소켓 십구 밀리", "소켓 열아홉 미리",
        "19미리 소켓", "십구미리 소켓", "열아홉 미리 소켓",
        "복스 소켓", "복스소켓", "복스알", "복스 알",
        "복스 19미리", "복스알 19미리", "복스 소켓 19미리",
        "박스 소켓", "박스소켓", "박스 알", "박스알", "박스 19미리",
        "복스 렌치", "박스 렌치", "소켓 렌치", "라쳇 소켓", "라쳇 렌치",
        "socket", "socket 19", "socket nineteen",
        "socket nineteen millimeter", "socket nineteen millimeters",
        "socket 19 mm", "socket 19 millimeter", "socket wrench",
        "sock it", "suck it", "soccer", "soccer 19",
        "소켓이", "소켓을", "소켓으로",
        "소켓 9미리", "소켓 90미리", "소켓 18미리", "소켓 17미리",
        "소켓 16미리", "소켓 20미리",
        "소켓 십육미리", "소켓 십칠미리", "소켓 십팔미리", "소켓 이십미리",
        "19미리 렌치", "십구미리 렌치", "렌치 19미리",
    ],
}

# 특정 공구를 가리키지 않는 일반 표현. 이 표현만 있으면 바로 실행하지 않고
# 어떤 공구인지 다시 묻는다.
GENERIC_TOOL_ALIASES = {
    "tool",
    "tools",
    "툴",
    "투울",
    "툴 줘",
    "공구",
    "공구 줘",
    "공구 가져와",
    "공구 찾아줘",
}

# 공구 후보로는 볼 수 있지만 사람 확인 없이 실행하면 위험한 표현.
# 예: "스캐너"는 "스패너" 오인식일 수 있지만 실제로는 다른 물체일 수도 있다.
UNCERTAIN_ALIASES = {
    "screwdriver": {
        "프린터 드라이버",
        "그래픽 드라이버",
        "장치 드라이버",
        "운전기사",
    },
    "multi_tool": {
        "multi two",
        "multi to",
        "multi too",
        "multi tu",
        "multi tour",
        "멀티탭",
        "멀티미터",
        "multimeter",
        "멀티미디어",
        "멀티플레이",
        "player",
    },
    "spanner_16mm": {
        "scanner",
        "스캐너",
    },
    "ratchet_wrench": {
    },
    "socket_19mm": {
        "sock it",
        "suck it",
        "soccer",
    },
}

# 치수 힌트. 프로젝트의 현재 공구 구성에서는 16mm는 스패너, 19mm는 복스 소켓을
# 강하게 가리킨다.
NUMBER_HINTS = {
    "spanner_16mm": ("16", "십육", "열여섯", "sixteen"),
    "socket_19mm": ("19", "십구", "열아홉", "nineteen"),
}

# 치수 충돌과 일반 렌치 요청 판정에 쓰는 공구군별 핵심 표현.
SPANNER_TERMS = (
    "스패너",
    "스파너",
    "스페너",
    "스페나",
    "스패나",
    "스페인어",
    "스페인",
    "spanish",
    "spanner",
    "spaner",
    "scanner",
    "스캐너",
)

SOCKET_TERMS = (
    "소켓",
    "복스",
    "박스 소켓",
    "박스소켓",
    "박스 알",
    "박스알",
    "socket",
    "sock it",
    "suck it",
    "soccer",
)

RATCHET_TERMS = (
    "라쳇",
    "라체트",
    "라챗",
    "라칫",
    "래칫",
    "래쳇",
    "라켓",
    "로켓",
    "ratchet",
    "rachet",
    "racket",
    "rocket",
    "런치",
    "렌즈",
)

GENERIC_WRENCH_TERMS = ("렌치", "렌찌", "랜치", "wrench")
