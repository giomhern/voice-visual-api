#!/usr/bin/env python3
"""Runner for the Stretch control API"""
import os
from control_api import create_app

HOST = os.getenv("HOST", "0.0.0.0")
PORT = int(os.getenv("PORT", "8080"))

app = create_app()

if __name__ == "__main__":
    print(f"[stretch-api] running on {HOST}:{PORT}")
    app.run(host=HOST, port=PORT, debug=False)