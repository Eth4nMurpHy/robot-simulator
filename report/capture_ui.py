"""Screenshots of the running web app for the report (needs Playwright and a Chromium/Chrome binary).

Start the app first (.venv/bin/python app.py --port 8765), then:
    .venv/bin/python report/capture_ui.py --chromium /usr/bin/google-chrome-stable
Loads the case-study instance of report/case_study.py (12 orders, seed 6, per-robot random SOC, 25 % threshold), runs CP-SAT with the same
deterministic single-worker options, and saves results/ui_replay.png and results/ui_analytics.png.
"""
import argparse
import json
from pathlib import Path
from playwright.sync_api import sync_playwright, expect

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--url', default='http://127.0.0.1:8765')
parser.add_argument('--chromium', default=None)
parser.add_argument('--at', type=int, default=489, help='replay time (s) of the simulation screenshot')
args = parser.parse_args()


def deterministic(route):
    body = json.loads(route.request.post_data)
    if body.get('strategy') == 'cpsat':
        body['custom_strategy'] = dict(workers=1, deterministic_time=3, time_limit=120)
    route.continue_(post_data=json.dumps(body))


def set_value(page, selector, value):
    page.locator(selector).evaluate('(el, v) => {el.value = v; el.dispatchEvent(new Event("input")); '
                                    'el.dispatchEvent(new Event("change"))}', str(value))


with sync_playwright() as p:
    browser = p.chromium.launch(executable_path=args.chromium, headless=True, args=['--no-sandbox'])
    page = browser.new_page(viewport={'width': 1440, 'height': 1000}, device_scale_factor=1.5)
    page.route('**/api/simulate', deterministic)
    page.goto(args.url)
    expect(page.locator('#task-source')).to_contain_text('generated tasks')
    for selector, value in [('#task-count', 12), ('#seed', 6), ('#release-gap', 25)]:
        set_value(page, selector, value)
    page.click('#generate')
    expect(page.locator('#task-source')).to_contain_text('12 generated tasks, seed 6')
    if not page.locator('#random-soc').is_checked():
        page.locator('#random-soc').check()
    set_value(page, '#safety', 25)
    set_value(page, '#chargers', 2)
    page.locator('#strategy').select_option('cpsat')
    page.click('#run')
    expect(page.locator('#m-completed')).to_have_text('12 / 12', timeout=180000)
    page.locator('#scrubber').evaluate('(el, v) => {el.value = v; el.dispatchEvent(new Event("input"))}', args.at)
    page.wait_for_timeout(300)
    page.evaluate('window.scrollTo(0, 0)')
    page.screenshot(path=str(ROOT / 'results' / 'ui_replay.png'), full_page=True)
    page.click('[data-tab="analytics"]')
    expect(page.locator('#gantt svg')).to_be_visible()
    page.wait_for_timeout(300)
    page.locator('#analytics').screenshot(path=str(ROOT / 'results' / 'ui_analytics.png'))
    browser.close()
