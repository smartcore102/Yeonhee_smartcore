from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
import crud, databases

router = APIRouter(tags=["users"])

@router.get("/users")
def get_users(db: Session = Depends(databases.get_db)):
    return crud.get_users(db)

@router.delete("/users/{user_id}")
def delete_user(user_id: int, db: Session = Depends(databases.get_db)):
    result = crud.delete_user(db, user_id)
    return {"deleted": result}