"""Dependency-free local web server for the robot simulator."""
import argparse
import json
import logging
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from simulator import InputError, generate_tasks, simulate

STATIC = Path(__file__).resolve().parent / 'static'


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(STATIC), **kwargs)

    def json_response(self, payload, status=200):
        encoded = json.dumps(payload, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(encoded)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(encoded)

    def do_GET(self):
        if self.path == '/api/health':
            self.json_response({'status': 'ok'})
        else:
            super().do_GET()

    def do_POST(self):
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 2_000_000:
                raise InputError('Request must be nonempty and smaller than 2 MB.')
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict):
                raise InputError('Request must be a JSON object.')
            if self.path == '/api/generate':
                self.json_response({'tasks': generate_tasks(data.get('count', 40), data.get('seed', 42), data.get('a_share', .5))})
            elif self.path == '/api/simulate':
                self.json_response(simulate(data.get('tasks'), data.get('config'), data.get('strategy', 'joint')))
            elif self.path == '/api/compare':
                rows = []
                for strategy in ('fcfs', 'deadline', 'joint'):
                    result = simulate(data.get('tasks'), data.get('config'), strategy)
                    rows.append({'strategy': strategy, 'label': result['strategy_label'], **result['summary']})
                self.json_response({'comparison': rows})
            else:
                self.json_response({'error': 'Unknown endpoint.'}, 404)
        except (InputError, json.JSONDecodeError, ValueError) as exc:
            self.json_response({'error': str(exc)}, 400)
        except Exception:
            logging.exception('Simulation request failed')
            self.json_response({'error': 'Unexpected server error. Check the server log.'}, 500)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8000)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f'Robot simulator listening on {args.host}:{args.port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.server_close()
