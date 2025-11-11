/**
 * 사용자 삭제 (Soft Delete) 요청 처리 함수
 * @param {number} userId 삭제할 사용자의 ID
 */
function deleteUserStatus(userId) {
    // 삭제 전에 사용자에게 확인 메시지 표시
    // Canvas 환경에서는 window.confirm() 대신 커스텀 모달 사용이 권장되나, 현재 로직을 유지
    if (!confirm("정말로 이 사용자를 삭제(비활성화)하시겠습니까? 삭제된 계정은 사용자 목록에서 사라집니다.")) {
        return;
    }

    // PUT 요청으로 status 업데이트 API 호출 (main.py의 /users/{user_id}/delete_status)
    fetch(`/users/${userId}/delete_status`, {
        method: 'PUT',
        headers: {
            'Content-Type': 'application/json',
        },
    })
    .then(response => {
        if (response.status === 200) {
            // 성공적으로 상태가 변경된 경우, 성공 메시지와 함께 페이지 새로고침
            window.location.href = "/users?success=사용자 계정이 성공적으로 비활성화되어 목록에서 제거되었습니다.";
        } else if (response.status === 404) {
            // 사용자를 찾을 수 없는 경우 에러 메시지 전달
            window.location.href = "/users?error=오류: 사용자를 찾을 수 없습니다.";
        } else {
            // 기타 오류 처리
            return response.json().then(data => {
                window.location.href = `/users?error=계정 삭제 실패: ${data.detail || '알 수 없는 오류'}`;
            });
        }
    })
    .catch(error => {
        console.error('Fetch error:', error);
        window.location.href = "/users?error=네트워크 오류 또는 서버 요청 중 문제가 발생했습니다.";
    });
}