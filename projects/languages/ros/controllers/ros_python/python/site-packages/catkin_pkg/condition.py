# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import operator

import pyparsing


def evaluate_condition(condition, context):
    if condition is None:
        return True
    expr = _get_condition_expression()
    try:
        parse_results = expr.parseString(condition, parseAll=True)
    except pyparsing.ParseException as e:
        raise ValueError(
            "condition '%s' failed to parse: %s" % (condition, e))
    return _evaluate(parse_results.asList()[0], context)


_condition_expression = None


def _get_condition_expression():
    global _condition_expression
    if not _condition_expression:
        pp = pyparsing
        operator = pp.Regex('==|!=|>=|>|<=|<').setName('operator')
        identifier = pp.Word('$', pp.alphanums + '_', min=2)
        value = pp.Word(pp.alphanums + '_-')
        comparison_term = identifier | value
        condition = pp.Group(comparison_term + operator + comparison_term)
        _condition_expression = pp.operatorPrecedence(
            condition, [
                ('and', 2, pp.opAssoc.LEFT, ),
                ('or', 2, pp.opAssoc.LEFT, ),
            ])
    return _condition_expression


def _evaluate(parse_results, context):
    if not isinstance(parse_results, list):
        if parse_results.startswith('$'):
            # get variable from context
            return str(context.get(parse_results[1:], ''))
        # return literal value
        return parse_results

    # recursion
    assert len(parse_results) == 3

    # handle logical operators
    if parse_results[1] == 'and':
        return _evaluate(parse_results[0], context) and \
            _evaluate(parse_results[2], context)
    if parse_results[1] == 'or':
        return _evaluate(parse_results[0], context) or \
            _evaluate(parse_results[2], context)

    # handle comparison operators
    operators = {
        '==': operator.eq,
        '!=': operator.ne,
        '<=': operator.le,
        '<': operator.lt,
        '>=': operator.ge,
        '>': operator.gt,
    }
    assert parse_results[1] in operators.keys()
    return operators[parse_results[1]](
        _evaluate(parse_results[0], context),
        _evaluate(parse_results[2], context))
