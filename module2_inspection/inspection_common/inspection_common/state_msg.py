"""노드 상태 문자열. 채현 님 /mission/status 형식("상태 | 이유 | 값")과 같은 구분자를 쓴다."""

SEP = " | "


def format_state(state: str, *fields: object) -> str:
    """첫 칸은 대문자 한 단어 상태, 나머지는 자유 문자열. '|'는 칸 안에 못 넣는다."""
    if not state or state != state.upper() or " " in state or "|" in state:
        raise ValueError(f"상태는 대문자 한 단어: {state!r}")
    texts = [str(f) for f in fields]
    for t in texts:
        if "|" in t:
            raise ValueError(f"칸에 '|'를 넣을 수 없음: {t!r}")
    return SEP.join([state, *texts])


def parse_state(text: str) -> tuple[str, list[str]]:
    parts = [p.strip() for p in text.split("|")]
    return parts[0], parts[1:]
