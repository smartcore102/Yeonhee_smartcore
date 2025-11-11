from passlib.context import CryptContext
from databases import SessionLocal # database.py에 정의된 SessionLocal을 임포트한다고 가정
from typing import Optional
from sqlalchemy.orm import Session
from sqlalchemy import Engine # Engine 타입을 사용하기 위해 임포트

# Bcrypt의 최대 비밀번호 입력 길이 (바이트 기준).
BCRYPT_MAX_LENGTH = 72

# CryptContext 초기화
# 환경 충돌이 잦은 'bcrypt' 대신, 기본 라이브러리로 처리 가능한 
# 'pbkdf2_sha256'을 사용. 이는 추가 백엔드 설치 문제를 우회함
pwd_context = CryptContext(schemes=["pbkdf2_sha256", "sha256_crypt"], default="pbkdf2_sha256", deprecated="auto")

def truncate_password(password: str) -> str:
    # pbkdf2_sha256은 길이 제한이 훨씬 덜 엄격하므로 이 로직은 안전 조치로만 작동
    encoded_password = password.encode('utf-8')
    if len(encoded_password) > BCRYPT_MAX_LENGTH:
        print("Warning: Password exceeds 72 bytes and will be truncated.")
        # 72바이트까지만 자르고 다시 문자열로 디코딩
        return encoded_password[:BCRYPT_MAX_LENGTH].decode('utf-8', 'ignore')
    return password

def hash_password(password: str) -> str:
    # 비밀번호를 해시하여 저장
    # pbkdf2_sha256은 길이 제한이 덜 엄격하므로, truncate 호출 없이 password를 그대로 전달
    return pwd_context.hash(password)

def verify_password(plain_password: str, hashed_password: str) -> bool:
    # 평문 비밀번호와 해시된 비밀번호를 비교
    try:
        # pbkdf2_sha256은 길이 제한이 덜 엄격하므로, password를 그대로 전달
        # 기존에 저장된 sha256_crypt 해시도 이 함수에서 자동으로 검증됨
        return pwd_context.verify(plain_password, hashed_password)
    except Exception as e:
        print(f"Error during password verification: {e}")
        return False

# DB 세션 의존성 주입 함수
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()