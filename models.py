from datetime import datetime
from sqlalchemy import Column, Integer, String, DateTime, func
from databases import Base

# Base = declarative_base() 

class User(Base):
    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    employee_number = Column(String(20), unique=True, index=True, nullable=False)
    username = Column(String(50), unique=True, index=True, nullable=False)
    hashed_password = Column(String, nullable=False)
    name = Column(String(50), nullable=False)
    role = Column(String(20), default="user", nullable=False) # admin, user 등
    
    # Soft Delete 상태를 'active' 또는 'deleted' 문자열로 관리
    status = Column(String(10), default="active", nullable=False) 

    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)
    
    def __repr__(self):
        return f"<User(id={self.id}, employee_number='{self.employee_number}', name='{self.name}', status='{self.status}')>"