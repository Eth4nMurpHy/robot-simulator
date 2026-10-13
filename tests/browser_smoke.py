"""Optional end-to-end check: pip install playwright, then run against app.py."""
import argparse
import json
import tempfile
from pathlib import Path
from playwright.sync_api import sync_playwright, expect

parser = argparse.ArgumentParser()
parser.add_argument('--url', default='http://127.0.0.1:8000')
parser.add_argument('--chromium', default=None)
parser.add_argument('--screenshots', default=None)
args = parser.parse_args()

with sync_playwright() as p, tempfile.TemporaryDirectory() as tmp:
    browser = p.chromium.launch(executable_path=args.chromium, headless=True, args=['--no-sandbox'])
    page = browser.new_page(viewport={'width': 1440, 'height': 1100}, accept_downloads=True)
    errors = []
    page.on('pageerror', lambda e: errors.append(str(e)))
    page.goto(args.url)
    expect(page.locator('#task-source')).to_contain_text('40 generated tasks')
    page.click('#run')
    expect(page.locator('#m-completed')).to_have_text('40 / 40')
    expect(page.locator('#m-safety')).to_have_text('0')
    page.click('#play')
    page.wait_for_function('current > 0')
    page.click('#play')
    before = page.evaluate('current')
    page.click('#step')
    assert page.evaluate('current') > before
    page.locator('#scrubber').evaluate('(el)=>{el.value=700;el.dispatchEvent(new Event("input"))}')
    assert page.evaluate('current') == 700
    assert page.locator('#clock').inner_text() == '11:40'
    if args.screenshots:
        Path(args.screenshots).mkdir(parents=True, exist_ok=True)
        page.screenshot(path=str(Path(args.screenshots) / 'simulation.png'), full_page=True)
    page.click('#reset')
    assert page.evaluate('current') == 0
    page.click('[data-tab="analytics"]')
    expect(page.locator('#gantt svg')).to_be_visible()
    expect(page.locator('#battery-chart svg')).to_be_visible()
    page.click('#compare')
    expect(page.locator('#comparison tbody tr')).to_have_count(3)
    if args.screenshots:
        page.screenshot(path=str(Path(args.screenshots) / 'analytics.png'), full_page=True)
    for button, filename, header in [('#export-schedule', 'schedule.csv', 'robot_id,task_id,action'),
                                      ('#export-events', 'events.csv', 'time,robot_id,event'),
                                      ('#export-summary', 'summary.csv', 'strategy,makespan')]:
        with page.expect_download() as download_info:
            page.click(button)
        download = download_info.value
        assert download.suggested_filename == filename
        destination = Path(tmp) / filename
        download.save_as(destination)
        assert destination.read_text().startswith(header)
    with page.expect_download() as download_info:
        page.click('#export-all')
    destination = Path(tmp) / 'results.json'
    download_info.value.save_as(destination)
    exported = json.loads(destination.read_text())
    assert exported['summary']['completed'] == 40
    assert len(exported['comparison']) == 3
    page.click('[data-tab="tasks"]')
    expect(page.locator('#task-table tbody tr')).to_have_count(40)
    page.fill('#task-search', 'T001')
    expect(page.locator('#task-table tbody tr')).to_have_count(1)
    page.fill('#task-search', '')
    page.click('#add-task')
    expect(page.locator('#task-table tbody tr')).to_have_count(41)
    page.locator('[data-delete="40"]').click()
    expect(page.locator('#task-table tbody tr')).to_have_count(40)
    task = dict(task_id='<img src=x onerror=alert(1)>', release_time=0, pickup='A1',
                dropoff='A2', load_time=15, unload_time=15, due_time=300, priority=2)
    page.locator('#file-input').set_input_files({'name': 'custom.json', 'mimeType': 'application/json', 'buffer': json.dumps([task]).encode()})
    expect(page.locator('#task-table tbody tr')).to_have_count(1)
    expect(page.locator('#notice')).to_contain_text('Imported 1 tasks')
    assert page.locator('#task-table img').count() == 0
    page.click('[data-tab="simulation"]')
    page.click('#run')
    expect(page.locator('#m-completed')).to_have_text('1 / 1')
    assert page.evaluate('result.summary.makespan') == 50
    page.click('[data-tab="tasks"]')
    task['task_id'] = 'CSV-001'
    fields = list(task)
    csv = ','.join(fields) + '\n' + ','.join(str(task[k]) for k in fields) + '\n'
    page.locator('#file-input').set_input_files({'name': 'custom.csv', 'mimeType': 'text/csv', 'buffer': csv.encode()})
    expect(page.locator('#task-source')).to_contain_text('custom.csv')
    page.locator('#file-input').set_input_files({'name': 'bad.csv', 'mimeType': 'text/csv', 'buffer': b'invalid,header\n1,2\n'})
    expect(page.locator('#notice')).to_contain_text('CSV must include all eight')
    expect(page.locator('#task-table tbody tr')).to_have_count(1)
    page.locator('[data-key="dropoff"]').select_option('A1')
    page.click('[data-tab="simulation"]')
    page.click('#run')
    expect(page.locator('#notice')).to_contain_text('distinct pickup/dropoff stations')
    assert not page.locator('#run').is_disabled()
    # Custom map and JSON strategy end-to-end.
    page.click('[data-tab="tasks"]')
    page.locator('[data-key="dropoff"]').select_option('A2')
    page.click('[data-tab="map-editor"]')
    expect(page.locator('#edge-rows tr')).to_have_count(18)
    page.fill('[data-edge="1"]', '40')
    page.click('#apply-map')
    expect(page.locator('#map-edit-status')).to_have_text('Applied')
    with page.expect_download() as download_info:
        page.click('#export-map')
    destination = Path(tmp) / 'map.json'
    download_info.value.save_as(destination)
    world = json.loads(destination.read_text())
    assert world['edges'][1]['length_m'] == 40
    world['edges'][1]['length_m'] = 30
    page.locator('#map-file').set_input_files({'name': 'map.json', 'mimeType': 'application/json', 'buffer': json.dumps(world).encode()})
    expect(page.locator('[data-edge="1"]')).to_have_value('30')
    if args.screenshots:
        page.screenshot(path=str(Path(args.screenshots) / 'map-editor.png'), full_page=True)
    page.click('[data-tab="strategy-editor"]')
    page.fill('#rule-name', 'My priority experiment')
    page.fill('#rule-robots', 'R2, R1, R4, R3')
    page.click('#apply-strategy')
    expect(page.locator('#strategy')).to_have_value('custom')
    with page.expect_download() as download_info:
        page.click('#export-strategy')
    destination = Path(tmp) / 'strategy.json'
    download_info.value.save_as(destination)
    rules = json.loads(destination.read_text())
    assert rules['preferred_robots'][0] == 'R2'
    rules['name'] = 'Imported experiment'
    page.locator('#strategy-file').set_input_files({'name': 'strategy.json', 'mimeType': 'application/json', 'buffer': json.dumps(rules).encode()})
    expect(page.locator('#rule-name')).to_have_value('Imported experiment')
    page.fill('#strategy-json', '{"name":"incomplete"}')
    page.click('#apply-strategy-json')
    expect(page.locator('#notice')).to_contain_text('must contain')
    if args.screenshots:
        page.screenshot(path=str(Path(args.screenshots) / 'strategy-editor.png'), full_page=True)
    page.click('[data-tab="simulation"]')
    page.click('#run')
    page.wait_for_function('result.strategy === "custom"')
    assert page.evaluate('result.summary.makespan') == 60
    assert page.evaluate('result.summary.energy') == 4
    assert page.evaluate('result.tasks[0].robot_id') == 'R2'
    assert page.locator('#edge-time').is_disabled()
    page.click('[data-tab="analytics"]')
    page.click('#compare')
    expect(page.locator('#comparison tbody tr')).to_have_count(4)
    page.reload()
    expect(page.locator('#map-source')).to_contain_text('Custom map')
    expect(page.locator('#strategy')).to_have_value('custom')
    page.click('[data-tab="strategy-editor"]')
    expect(page.locator('#rule-name')).to_have_value('Imported experiment')
    page.click('[data-tab="simulation"]')
    page.locator('#strategy').select_option('joint')
    page.reload()
    expect(page.locator('#map-source')).to_contain_text('Custom map')
    expect(page.locator('#strategy')).to_have_value('joint')
    page.set_viewport_size({'width': 390, 'height': 844})
    page.click('[data-tab="guide"]')
    assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth')
    page.click('[data-tab="simulation"]')
    assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth')
    if args.screenshots:
        page.screenshot(path=str(Path(args.screenshots) / 'mobile.png'), full_page=True)
    assert not errors, errors
    browser.close()
    print('Browser smoke passed: replay, charts, comparisons, CSV/JSON import/export, task/map/strategy editors, saved settings, invalid inputs, mobile, and XSS escaping.')
