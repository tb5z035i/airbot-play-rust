# /// script
# dependencies = [
#   "websockets>=15.0",
# ]
# ///

"""Explicit websocket-named entrypoint for the AIRBOT keyboard UI.

Usage examples:

    uv run scripts/airbot_play_ws_keyboard.py --url ws://127.0.0.1:9002

    uv run scripts/airbot_play_ws_keyboard.py --spawn-server --interface can1
"""

from __future__ import annotations

import asyncio

from airbot_play_keyboard import main


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
