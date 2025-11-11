const API_URL = "http://127.0.0.1:8000";  // EC2 서버 IP

document.getElementById("loginBtn").addEventListener("click", async () => {
  const username = document.getElementById("username").value;
  const password = document.getElementById("password").value;

  const formData = new FormData(document.getElementById('login-form'));

  const res = await fetch(`${API_URL}/login`, {
    method: "POST",
    body: new URLSearchParams(formData),
  });

  if (res.ok) {
    const data = await res.json();
    localStorage.setItem("token", data.access_token);
    window.location.href = "main.html";
  } else {
    document.getElementById("message").innerText = "로그인 실패!";
  }
});