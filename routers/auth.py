from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
import schemas, crud, databases

router = APIRouter(tags=["auth"])

@router.post("/login")
def login(data: schemas.LoginSchema, db: Session = Depends(databases.get_db)):
    user = crud.authenticate_user(db, data.username, data.password)
    if not user:
        raise HTTPException(status_code=400, detail="Invalid credentials")
    return {"access_token": "fake-jwt-token", "user_id": user.id}

@router.post("/register", response_model=schemas.UserOut)
def register(user: schemas.UserCreate, db: Session = Depends(databases.get_db)):
    user.hashed_password = crud.hash_password(user.hashed_password)
    return crud.create_user(db, user)