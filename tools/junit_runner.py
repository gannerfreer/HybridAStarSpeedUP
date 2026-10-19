#!/usr/bin/env python3
"""标准库 JUnit XML 测试运行器（CPython 3.8+，无第三方依赖）。

``tools/task_runner.py test --component tools`` 通过本文件复现计划中的
``python3 -m unittest discover -s tools/tests -p 'test_*.py'`` 发现逻辑，
并用标准库 ``xml.etree.ElementTree`` 生成 ``junit.xml``。

退出码：0 = 全部通过且至少运行 1 个用例；1 = 存在失败/错误或无用例；
2 = 参数错误（argparse 约定）。
"""

import argparse
import os
import sys
import time
import unittest
from datetime import datetime, timezone
from xml.etree import ElementTree as ET


def _utc_stamp():
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


class JUnitTestResult(unittest.TestResult):
    """记录每个用例的结果与耗时，供 XML 生成使用。"""

    def __init__(self, stream=None, descriptions=None, verbosity=None):
        super(JUnitTestResult, self).__init__(stream, descriptions, verbosity)
        self.outcomes = {}
        self.durations = {}
        self._started = {}

    def startTest(self, test):
        super(JUnitTestResult, self).startTest(test)
        self._started[test.id()] = time.monotonic()

    def stopTest(self, test):
        started = self._started.pop(test.id(), None)
        if started is not None:
            self.durations[test.id()] = max(0.0, time.monotonic() - started)
        super(JUnitTestResult, self).stopTest(test)

    def _record(self, test, outcome, message=None, detail=None):
        self.outcomes[test.id()] = (outcome, message, detail)

    def _format_error(self, test, err):
        message = str(err[1]) if err and len(err) > 1 and err[1] is not None else ""
        try:
            detail = self._exc_info_to_string(err, test)
        except Exception:  # noqa: BLE001 - 保底：不能因为格式化失败影响测试运行
            detail = message
        return message, detail

    def addSuccess(self, test):
        super(JUnitTestResult, self).addSuccess(test)
        self._record(test, "success")

    def addFailure(self, test, err):
        super(JUnitTestResult, self).addFailure(test, err)
        message, detail = self._format_error(test, err)
        self._record(test, "failure", message, detail)

    def addError(self, test, err):
        super(JUnitTestResult, self).addError(test, err)
        message, detail = self._format_error(test, err)
        self._record(test, "error", message, detail)

    def addSkip(self, test, reason):
        super(JUnitTestResult, self).addSkip(test, reason)
        self._record(test, "skipped", str(reason))

    def addExpectedFailure(self, test, err):
        super(JUnitTestResult, self).addExpectedFailure(test, err)
        message, detail = self._format_error(test, err)
        self._record(test, "skipped", "expected failure: %s" % message)

    def addUnexpectedSuccess(self, test):
        super(JUnitTestResult, self).addUnexpectedSuccess(test)
        self._record(test, "failure", "unexpected success")

    def addSubTest(self, test, subtest, err):
        super(JUnitTestResult, self).addSubTest(test, subtest, err)
        if err is not None:
            if issubclass(err[0], test.failureException):
                message, detail = self._format_error(test, err)
                self._record(test, "failure", message, detail)
            else:
                message, detail = self._format_error(test, err)
                self._record(test, "error", message, detail)


def _split_test_id(test_id):
    if "." in test_id:
        classname, name = test_id.rsplit(".", 1)
    else:
        classname, name = test_id, test_id
    return classname, name


def build_xml(result, suite_name):
    total_time = sum(result.durations.values())
    tests_node = ET.Element("testsuites", {
        "name": suite_name,
        "tests": str(len(result.outcomes)),
        "failures": str(len(result.failures)),
        "errors": str(len(result.errors)),
        "skipped": str(len(result.skipped)),
        "time": "%.6f" % total_time,
    })
    suite_node = ET.SubElement(tests_node, "testsuite", {
        "name": suite_name,
        "tests": str(len(result.outcomes)),
        "failures": str(len(result.failures)),
        "errors": str(len(result.errors)),
        "skipped": str(len(result.skipped)),
        "time": "%.6f" % total_time,
        "timestamp": _utc_stamp(),
    })
    for test_id, outcome_tuple in result.outcomes.items():
        outcome, message, detail = outcome_tuple
        classname, name = _split_test_id(test_id)
        case = ET.SubElement(suite_node, "testcase", {
            "classname": classname,
            "name": name,
            "time": "%.6f" % result.durations.get(test_id, 0.0),
        })
        if outcome == "failure":
            node = ET.SubElement(case, "failure",
                                 {"message": message or "", "type": "Failure"})
            node.text = detail or ""
        elif outcome == "error":
            node = ET.SubElement(case, "error",
                                 {"message": message or "", "type": "Error"})
            node.text = detail or ""
        elif outcome == "skipped":
            node = ET.SubElement(case, "skipped", {"message": message or ""})
    return tests_node


def write_xml(path, root_node):
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    body = ET.tostring(root_node, encoding="unicode")
    with open(path, "w", encoding="utf-8") as handle:
        handle.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        handle.write(body)
        handle.write("\n")


def main(argv=None):
    parser = argparse.ArgumentParser(prog="junit_runner.py",
                                     description="stdlib unittest + JUnit XML")
    parser.add_argument("--start-dir", required=True, help="测试发现起始目录")
    parser.add_argument("--pattern", default="test_*.py", help="测试文件名模式")
    parser.add_argument("--junit", required=True, help="JUnit XML 输出路径")
    parser.add_argument("--repo-root", default=None,
                        help="仓库根目录（默认由脚本位置推导），用于导入 tools 包")
    args = parser.parse_args(argv)

    repo_root = args.repo_root or os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)

    start_dir = os.path.abspath(args.start_dir)
    suite_name = os.path.basename(start_dir) or "tests"

    loader = unittest.TestLoader()
    try:
        suite = loader.discover(start_dir, pattern=args.pattern,
                                top_level_dir=start_dir)
    except Exception as exc:  # noqa: BLE001 - 发现阶段失败也要留下 XML
        root_node = ET.Element("testsuites", {"name": suite_name, "tests": "0",
                                              "failures": "0", "errors": "1",
                                              "skipped": "0", "time": "0.000000"})
        suite_node = ET.SubElement(root_node, "testsuite", {"name": suite_name})
        case = ET.SubElement(suite_node, "testcase", {"classname": "discovery",
                                                      "name": "discover", "time": "0"})
        node = ET.SubElement(case, "error", {"message": str(exc), "type": "DiscoveryError"})
        node.text = str(exc)
        write_xml(args.junit, root_node)
        sys.stderr.write("测试发现失败: %s\n" % exc)
        return 1

    runner = unittest.TextTestRunner(stream=sys.stderr, verbosity=2,
                                     resultclass=JUnitTestResult)
    result = runner.run(suite)
    if not isinstance(result, JUnitTestResult):
        sys.stderr.write("内部错误: 结果类型异常 %r\n" % type(result))
        return 1
    write_xml(args.junit, build_xml(result, suite_name))
    if result.testsRun == 0:
        sys.stderr.write("未发现任何测试用例（pattern=%s, start-dir=%s）\n"
                         % (args.pattern, start_dir))
        return 1
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())
