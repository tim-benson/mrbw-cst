"""Offline tests for check_layout_change_bumps_version.py's pure diff logic - no git/filesystem needed.

Run with: python3 -m unittest discover -s src/cst-cfgtransfer/tests -t src/cst-cfgtransfer
       or: cd src/cst-cfgtransfer && python3 -m unittest discover tests
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from check_layout_change_bumps_version import (  # noqa: E402
    check_layout_change_requires_bump,
    _extract_defines,
)

BASE = """\
#ifndef _CST_EEPROM_H_
#define _CST_EEPROM_H_

#include <stdint.h>

#define EE_VERSION_MAJOR              0x0E

#define EEPROM_LAYOUT_VERSION          2

#define EE_HORN_THRESHOLD             0x20
#define EE_BRAKE_THRESHOLD            0x21

#define CONFIG_OFFSET(cfgNum)         (((cfgNum - 1) * CONFIG_SIZE) + CONFIG_START)

#define STACK_COMBO_BRK1              0x08
#define STACK_3STEP_DEFAULT_1         STACK_COMBO_BRK2

#endif
"""


def _with(old, replacements):
    new = old
    for before, after in replacements:
        assert before in new, "fixture setup error: %r not found" % before
        new = new.replace(before, after, 1)
    return new


class CheckLayoutChangeRequiresBumpTests(unittest.TestCase):

    def test_no_change_passes(self):
        self.assertIsNone(check_layout_change_requires_bump(BASE, BASE))

    def test_comment_only_edit_passes(self):
        new = BASE + "\n// just a note, no #define touched\n"
        self.assertIsNone(check_layout_change_requires_bump(BASE, new))

    def test_value_changed_with_version_bump_passes(self):
        new = _with(BASE, [
            ("#define EE_HORN_THRESHOLD             0x20", "#define EE_HORN_THRESHOLD             0x22"),
            ("#define EEPROM_LAYOUT_VERSION          2", "#define EEPROM_LAYOUT_VERSION          3"),
        ])
        self.assertIsNone(check_layout_change_requires_bump(BASE, new))

    def test_value_changed_without_version_bump_fails(self):
        new = _with(BASE, [
            ("#define EE_HORN_THRESHOLD             0x20", "#define EE_HORN_THRESHOLD             0x22"),
        ])
        message = check_layout_change_requires_bump(BASE, new)
        self.assertIsNotNone(message)
        self.assertIn("EE_HORN_THRESHOLD", message)
        self.assertIn("0x20 -> 0x22", message)

    def test_define_added_without_version_bump_fails(self):
        new = BASE.replace(
            "#define EE_BRAKE_THRESHOLD            0x21",
            "#define EE_BRAKE_THRESHOLD            0x21\n#define EE_HORN2_FUNCTION             0x4D",
        )
        message = check_layout_change_requires_bump(BASE, new)
        self.assertIsNotNone(message)
        self.assertIn("added:   EE_HORN2_FUNCTION", message)

    def test_define_removed_without_version_bump_fails(self):
        new = BASE.replace("#define EE_BRAKE_THRESHOLD            0x21\n", "")
        message = check_layout_change_requires_bump(BASE, new)
        self.assertIsNotNone(message)
        self.assertIn("removed: EE_BRAKE_THRESHOLD", message)

    def test_version_only_bump_passes(self):
        new = BASE.replace(
            "#define EEPROM_LAYOUT_VERSION          2", "#define EEPROM_LAYOUT_VERSION          3"
        )
        self.assertIsNone(check_layout_change_requires_bump(BASE, new))

    def test_function_like_macro_change_without_bump_fails(self):
        new = BASE.replace(
            "#define CONFIG_OFFSET(cfgNum)         (((cfgNum - 1) * CONFIG_SIZE) + CONFIG_START)",
            "#define CONFIG_OFFSET(cfgNum)         (((cfgNum - 1) * CONFIG_SIZE) + CONFIG_START + 1)",
        )
        message = check_layout_change_requires_bump(BASE, new)
        self.assertIsNotNone(message)
        self.assertIn("changed: CONFIG_OFFSET", message)

    def test_non_layout_define_added_without_version_bump_passes(self):
        # A new value/encoding constant that is not an EE_* offset, CONFIG_*, or a structural size is
        # not a layout move - it must not force a version bump (this is the Commit-21 --no-verify case).
        new = BASE.replace(
            "#define STACK_COMBO_BRK1              0x08",
            "#define STACK_COMBO_BRK1              0x08\n#define STACK_COMBO_MASK             0x68",
        )
        self.assertIsNone(check_layout_change_requires_bump(BASE, new))

    def test_non_layout_define_value_changed_without_version_bump_passes(self):
        new = _with(BASE, [
            ("#define STACK_3STEP_DEFAULT_1         STACK_COMBO_BRK2",
             "#define STACK_3STEP_DEFAULT_1         STACK_COMBO_BRK3"),
        ])
        self.assertIsNone(check_layout_change_requires_bump(BASE, new))

    def test_include_after_guard_does_not_hide_a_layout_change(self):
        # Regression: the old \\s separator let "#define _CST_EEPROM_H_" swallow the next non-blank
        # line, so whether EE_VERSION_MAJOR parsed at all depended on the blank line / #include sitting
        # between it and the guard. A real EE_* change must still be caught regardless.
        no_include = BASE.replace("\n#include <stdint.h>\n", "")
        for base in (BASE, no_include):
            new = _with(base, [
                ("#define EE_HORN_THRESHOLD             0x20",
                 "#define EE_HORN_THRESHOLD             0x22"),
            ])
            message = check_layout_change_requires_bump(base, new)
            self.assertIsNotNone(message)
            self.assertIn("EE_HORN_THRESHOLD", message)
            self.assertIn("0x20 -> 0x22", message)

    def test_guard_line_is_never_parsed_as_a_define(self):
        defines = _extract_defines(BASE)
        self.assertNotIn("_CST_EEPROM_H_", defines)
        self.assertEqual(defines["EE_VERSION_MAJOR"], "0x0E")


if __name__ == "__main__":
    unittest.main()
