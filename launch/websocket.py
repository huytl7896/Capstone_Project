from flask import Flask, render_template_string
from flask_socketio import SocketIO, emit
import cv2
import numpy as np
import os
import datetime
import base64

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

# Lưu ảnh vào thư mục
base_path = "/home/huy/dev_ws/src/my_package/Image"
os.makedirs(os.path.join(base_path, "fire"), exist_ok=True)
os.makedirs(os.path.join(base_path, "person"), exist_ok=True)

# Bộ nhớ tạm thời
current_image = None
current_label = None


@app.route("/")
def index():
    return render_template_string("""
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Xác nhận ảnh (WebSocket)</title>
    <script src="https://cdn.socket.io/4.4.1/socket.io.min.js"></script>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: 'Segoe UI', Arial, sans-serif;
            background: linear-gradient(135deg, #e0eafc, #cfdef3);
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            justify-content: center;
            align-items: center;
            padding: 20px;
        }

        h1 {
            color: #2c3e50;
            font-size: 2.2rem;
            margin-bottom: 20px;
            text-shadow: 1px 1px 2px rgba(0, 0, 0, 0.1);
            animation: fadeIn 0.5s ease-in;
        }

        #image-box {
            max-width: 600px;
            width: 100%;
            margin-bottom: 20px;
            transition: all 0.3s ease;
        }

        img {
            width: 100%;
            max-height: 400px; /* Limit image height to reduce size */
            object-fit: contain; /* Ensure image scales properly */
            border-radius: 12px;
            box-shadow: 0 4px 15px rgba(0, 0, 0, 0.2);
            border: 3px solid #ffffff;
            transition: transform 0.3s ease;
        }

        img:hover {
            transform: scale(1.02);
        }

        #buttons {
            display: flex;
            gap: 15px;
        }

        button {
            padding: 12px 30px;
            font-size: 1.1rem;
            font-weight: 600;
            border: none;
            border-radius: 50px;
            cursor: pointer;
            transition: all 0.3s ease;
            box-shadow: 0 3px 10px rgba(0, 0, 0, 0.2);
        }

        #yes-btn {
            background: #27ae60;
            color: white;
        }

        #yes-btn:hover {
            background: #2ecc71;
            transform: translateY(-2px);
        }

        #no-btn {
            background: #c0392b;
            color: white;
        }

        #no-btn:hover {
            background: #e74c3c;
            transform: translateY(-2px);
        }

        button:active {
            transform: translateY(0);
            box            box-shadow: 0 2px 5px rgba(0, 0, 0, 0.2);
        }

        @keyframes fadeIn {
            from { opacity: 0; transform: translateY(-10px); }
            to { opacity: 1; transform: translateY(0); }
        }

        @media (max-width: 600px) {
            h1 {
                font-size: 1.8rem;
            }

            #image-box {
                max-width: 90%;
            }

            img {
                max-height: 300px; /* Smaller height for mobile devices */
            }

            button {
                padding: 10px 20px;
                font-size: 1rem;
            }
        }
    </style>
</head>
<body>
    <h1 id="title">⏳ Đang chờ ảnh...</h1>
    <div id="image-box"></div>
    <div id="buttons" style="display:none;">
        <button id="yes-btn">✅ Đúng</button>
        <button id="no-btn">❌ Sai</button>
    </div>

    <script>
        const socket = io();

        socket.on("new_image", (data) => {
            const audio = new Audio('https://www.soundjay.com/button/sounds/button-16.mp3');
            audio.play();
            document.getElementById("title").innerText = "Xác nhận đây là " + data.label + "?";
            document.getElementById("image-box").innerHTML = '<img src="' + data.image + '" alt="Ảnh">';
            document.getElementById("buttons").style.display = "flex";
        });

        socket.on("reset", () => {
            document.getElementById("title").innerText = "⏳ Đang chờ ảnh...";
            document.getElementById("image-box").innerHTML = "";
            document.getElementById("buttons").style.display = "none";
        });

        socket.on("error", (data) => {
            alert("Lỗi: " + data.message);
        });

        document.getElementById("yes-btn").onclick = () => {
            socket.emit("confirm", { action: "yes" });
        };

        document.getElementById("no-btn").onclick = () => {
            socket.emit("confirm", { action: "no" });
        };
    </script>
</body>
</html>
""")


@socketio.on("upload_image")
def handle_upload(data):
    global current_image, current_label
    label = data.get("label")
    image_data = data.get("image")

    if label not in ["fire", "person"] or not image_data:
        emit("error", {"message": "Thiếu ảnh hoặc label không hợp lệ."})
        return

    try:
        header, encoded = image_data.split(",", 1)
        img_bytes = base64.b64decode(encoded)
        img_array = np.frombuffer(img_bytes, np.uint8)
        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

        if img is not None:
            current_image = img
            current_label = label
            emit("new_image", {"label": label, "image": image_data}, broadcast=True)
            print(f"📷 Nhận ảnh mới: {label}")
        else:
            emit("error", {"message": "Không thể giải mã ảnh."})
    except Exception as e:
        emit("error", {"message": f"Lỗi xử lý ảnh: {str(e)}"})


@socketio.on("confirm")
def handle_confirm(data):
    global current_image, current_label
    action = data.get("action")

    if action == "yes" and current_image is not None and current_label in ["fire", "person"]:
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"confirmed_{timestamp}.jpg"
        save_path = os.path.join(base_path, current_label, filename)
        cv2.imwrite(save_path, current_image)
        print(f"✅ Đã lưu ảnh: {save_path}")
    else:
        print("❌ Không lưu ảnh.")

    current_image = None
    current_label = None
    emit("reset", {}, broadcast=True)


if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5000)
