from pydantic import BaseModel
from typing import Optional
from datetime import datetime

# Base 스키마 (공통 필드 정의)

class UserBase(BaseModel):
    employee_number: str
    username: str
    name: str
    role: str = "user"

# 사용자 생성 스키마 (비밀번호 포함)

class UserCreate(UserBase):
    password: str # 클라이언트로부터 입력받는 필드명

# 사용자 읽기 스키마 (DB 응답 포맷)

class UserRead(BaseModel):
    id: int
    employee_number: str
    username: str
    name: str
    role: str
    status: str
    created_at: datetime
    updated_at: datetime

    class Config:
        # Pydantic V2 권장 사항으로 orm_mode 대신 from_attributes 사용
        from_attributes = True

# 사용자 업데이트 스키마 (모든 필드는 선택 사항)

class UserUpdate(BaseModel):
    employee_number: Optional[str] = None
    username: Optional[str] = None
    password: Optional[str] = None
    name: Optional[str] = None
    role: Optional[str] = None
    status: Optional[str] = None

# 로그인 스키마 (현재 main.py는 Form 데이터를 사용하지만 참고용으로 유지)

class LoginSchema(BaseModel):
    username: str
    password: str

# 앱용ㅇ 로그인 요청/응답 스키마 추가
class LoginResponse(BaseModel):
    status: str
    role: Optional[str] = None
    message: Optional[str] = None

class LoginRequest(BaseModel):
    employee_number: str
    username: str
    password: str