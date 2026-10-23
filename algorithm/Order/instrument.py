"""Strict, auditable changes to local copies of the existing router."""
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
KINDS = ("syncnet", "tbsnet", "btsnet", "bbsnet", "btnet", "tbnet", "bbnet", "tsbsnet")
ORIGINAL = (0, 1, 1, 2, 3, 3, 4, 5)


def replace(text, old, new, count=1):
    if text.count(old) != count:
        raise RuntimeError(f"Instrumentation anchor changed: {old!r}")
    return text.replace(old, new)


def instrument(source):
    def edit(relative, transform):
        original = (ROOT / "source" / relative).read_text()
        (source / relative).write_text(transform(original))

    for index, (name, base) in enumerate(zip(KINDS, ORIGINAL)):
        edit(f"circuit/net/types/{name}.cc", lambda text, i=index, b=base:
             '#include "order_policy.hh"\n' + replace(text, f"Net{{Priority{{{b}}},",
                                                    f"Net{{Priority{{order_experiment::base_priority({i})}},"))

    def width(text):
        text, count = re.subn(r"COB_ARRAY_WIDTH\s*=\s*\d+", "COB_ARRAY_WIDTH = ORDER_EXPERIMENT_COB_WIDTH", text)
        if count != 1:
            raise RuntimeError("COB width anchor changed")
        return text
    edit("hardware/interposer.hh", width)

    def sort(text):
        text = replace(text, "return n1->priority() > n2->priority();", "return order_experiment::compare(n1, n2);")
        text = replace(text, "engine.update_net_seq(nets);",
                       "engine.update_net_seq(nets);\n        order_experiment::record_order(nets);")
        return '#include "observation.hh"\n' + text
    edit("algo/router/command_mode/command/sort.cc", sort)

    def route(text):
        text = replace(text, "        try {\n            net->route",
                       "        order_experiment::start_net(i, net);\n"
                       "        const auto experiment_start = order_experiment::Clock::now();\n"
                       "        bool threw_retry = false;\n        try {\n            net->route")
        text = replace(text, "catch (const RetryExpt& err) {", "catch (const RetryExpt& err) {\n            threw_retry = true;")
        text = replace(text, "        engine.move_on();",
                       "        order_experiment::finish_net(i, net, experiment_start, threw_retry);\n        engine.move_on();")
        return '#include "observation.hh"\n' + text
    edit("algo/router/command_mode/command/route.cc", route)

    # Preserve the primary COB-unit usage heuristic. Only equal-usage candidates
    # get a coordinate tie-break instead of pointer-hash iteration order.
    edit("algo/router/common/maze/mazeroutestrategy.cc", lambda text: replace(text,
         "return cobunit_usage[cobunit1] < cobunit_usage[cobunit2];",
         "if (cobunit_usage[cobunit1] != cobunit_usage[cobunit2])\n"
         "                return cobunit_usage[cobunit1] < cobunit_usage[cobunit2];\n"
         "            return track1->coord() < track2->coord();"))
    (source / "global/utility/random.cc").write_text(
        '#include "order_policy.hh"\n#include <utility/random.hh>\n'
        'namespace PR_tool {\n'
        'auto random() -> double { return order_experiment::random_real(); }\n'
        'auto random_i64(std::i64 min, std::i64 max) -> std::i64 {\n'
        '    return order_experiment::random_integer(min, max);\n}\n}\n')
