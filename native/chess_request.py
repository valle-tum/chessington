import sys
import requests
import json

def get_best_move(fen):
    url = "https://chess-stockfish-16-api.p.rapidapi.com/chess/api"

    payload = f"-----011000010111000001101001\r\nContent-Disposition: form-data; name=\"fen\"\r\n\r\n{fen}\r\n-----011000010111000001101001--\r\n\r\n"
    headers = {
        "x-rapidapi-key": "678035a1c2mshd447467226dc590p1afa71jsn5f0f21fdad45",
        "x-rapidapi-host": "chess-stockfish-16-api.p.rapidapi.com",
        "Content-Type": "multipart/form-data; boundary=---011000010111000001101001"
    }

    response = requests.post(url, data=payload, headers=headers)
    return response.json()

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 chess_request.py <fen>")
        sys.exit(1)

    fen = sys.argv[1]
    result = get_best_move(fen)
    print(json.dumps(result, indent=4))

if __name__ == "__main__":
    main()