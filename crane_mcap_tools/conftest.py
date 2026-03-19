def pytest_sessionfinish(session, exitstatus):
    """テストが収集されなかった場合（exit code 5）を正常終了にする."""
    if exitstatus == 5:
        session.exitstatus = 0
