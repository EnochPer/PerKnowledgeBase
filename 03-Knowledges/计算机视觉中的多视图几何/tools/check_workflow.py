"""非构建自检：python3 -B check_workflow.py；状态读写在临时目录完成。"""
import argparse
import json
import tempfile
from pathlib import Path

import mvg_pdf as m

assert m.physical_page(25, 18) == 43
assert m.physical_page(154, 18) == 172
assert m.page_link(154, 18).endswith('#page=172')
for invalid in (0, -1, 10000):
    try:
        m.physical_page(invalid, 18)
    except ValueError:
        pass
    else:
        raise AssertionError(f'非法页码未被拒绝：{invalid}')
original = m.STATE
with tempfile.TemporaryDirectory() as directory:
    m.STATE = Path(directory) / 'state.json'
    m.cmd_set(argparse.Namespace(book_page=154, offset=18, topic='投影', mastered=None))
    state = m.load_state()
    assert state['organized_topics'] == ['投影']
    assert state['mastered_topics'] == []
    state['pending_questions'] = ['待复习']
    m.STATE.write_text(json.dumps(state))
    m.cmd_set(argparse.Namespace(book_page=25, offset=18, topic=None, mastered=None))
    assert m.load_state()['pending_questions'] == ['待复习']
    assert m.load_state()['current']['pdf_page'] == 43
    m.STATE.write_text('{broken')
    try:
        m.load_state()
    except ValueError:
        pass
    else:
        raise AssertionError('损坏状态未报错')
m.STATE = original
print('页码边界、链接、状态初始化/切换/保留与损坏状态检查通过。')
