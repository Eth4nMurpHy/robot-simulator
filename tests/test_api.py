import json
import threading
import unittest
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from app import Handler
from simulator import generate_tasks


class QuietHandler(Handler):
    def log_message(self, *args):
        pass


class APITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = ThreadingHTTPServer(('127.0.0.1', 0), QuietHandler)
        cls.url = f'http://127.0.0.1:{cls.server.server_port}'
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()

    def post(self, path, data):
        request = Request(self.url + path, json.dumps(data).encode(), {'Content-Type': 'application/json'})
        with urlopen(request) as response:
            return json.load(response)

    def test_health_and_assets(self):
        with urlopen(self.url + '/api/health') as response:
            self.assertEqual(json.load(response)['status'], 'ok')
        for path in ('/', '/app.js', '/style.css'):
            with urlopen(self.url + path) as response:
                self.assertEqual(response.status, 200)
                self.assertGreater(len(response.read()), 100)

    def test_generate_simulate_and_compare(self):
        tasks = self.post('/api/generate', {'count': 20, 'seed': 42})['tasks']
        result = self.post('/api/simulate', {'tasks': tasks, 'strategy': 'joint'})
        self.assertEqual(result['summary']['completed'], 20)
        self.assertTrue(result['validation']['passed'])
        comparison = self.post('/api/compare', {'tasks': tasks})['comparison']
        self.assertEqual(len(comparison), 3)
        self.assertTrue(all(c['completed'] == 20 and c['violations'] == 0 for c in comparison))

    def test_customization_validation_and_comparison(self):
        from customization import DEFAULT_RULES, default_map
        with urlopen(self.url + '/api/options') as response:
            options = json.load(response)
        self.assertEqual(len(options['map']['edges']), 18)
        self.assertIn('weights', options['custom_strategy'])
        validated = self.post('/api/validate-customization', {'map': default_map(), 'custom_strategy': DEFAULT_RULES})
        request = {'tasks': generate_tasks(20), 'strategy': 'custom', **validated}
        result = self.post('/api/simulate', request)
        self.assertEqual(result['strategy'], 'custom')
        self.assertTrue(result['validation']['passed'])
        comparison = self.post('/api/compare', request)['comparison']
        self.assertEqual(len(comparison), 4)
        self.assertEqual(comparison[-1]['strategy'], 'custom')
        for data in ({'map': {'edges': []}}, {'map': None}, {'custom_strategy': {'name': 'bad'}}):
            with self.assertRaises(HTTPError) as context:
                self.post('/api/validate-customization', data)
            self.assertEqual(context.exception.code, 400)

    def test_invalid_payloads_are_400(self):
        for path, data in [('/api/simulate', {'tasks': []}), ('/api/generate', {'count': -1}), ('/api/simulate', {'tasks': generate_tasks(1), 'strategy': 'nope'}), ('/api/simulate', [])]:
            with self.subTest(path=path, data=data):
                with self.assertRaises(HTTPError) as context:
                    self.post(path, data)
                self.assertEqual(context.exception.code, 400)
                self.assertIn('error', json.load(context.exception))


if __name__ == '__main__':
    unittest.main()
