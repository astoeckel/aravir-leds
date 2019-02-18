#  Aravir -- Linux compatiable virtual I2C devices -- LED Controller
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


project('aravir-leds', 'cpp', default_options : ['cpp_std=c++14'])

# Include directory
inc_aravir_leds = include_directories(['.'])

# Compile and register the unit tests
dep_foxenunit = dependency(
    'libfoxenunit',
    fallback:['libfoxenunit', 'dep_foxenunit'])

exe_test_leds = executable(
    'test_leds',
    'test/test_leds.cpp',
    include_directories: inc_aravir_leds,
    dependencies: dep_foxenunit,
    install: false)
test('test_leds', exe_test_leds)

