#  Board management controller (BMC) for a battery powered RPi computer
#  Copyright (C) 2019  Andreas Stöckel
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Affero General Public License as
#  published by the Free Software Foundation, either version 3 of the
#  License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Affero General Public License for more details.
#
#  You should have received a copy of the GNU Affero General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.

project('rpibmc', 'cpp', default_options : ['cpp_std=c++14'])

add_project_arguments('-Wno-non-virtual-dtor', language : 'cpp')

# Include directory
inc_rpibmc = include_directories([
    '.',
    './lib/soft323x',
])

# Compile and register the unit tests
dep_foxenunit = dependency(
    'libfoxenunit',
    fallback:['libfoxenunit', 'dep_foxenunit'])

exe_test_led = executable(
    'test_led',
    'test/test_led.cpp',
    include_directories: inc_rpibmc,
    dependencies: dep_foxenunit,
    install: false)
test('test_led', exe_test_led)

exe_test_i2c_hub = executable(
    'test_i2c_hub',
    'test/test_i2c_hub.cpp',
    include_directories: inc_rpibmc,
    dependencies: dep_foxenunit,
    install: false)
test('test_i2c_hub', exe_test_i2c_hub)

