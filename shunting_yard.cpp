//
//  Shunting_Yard.cpp
//  Algorithm
//
//  Created by Ray Shen on 2015-07-22.
//
//  Implemented shunting yard algorithm for parsing logical expression
//  by transforming in-fix notation into reverse-Polish notation using stack.
//
//  Docode the transformed string using recognition pattern based on logical operation symbols.
// 

#include <string>
#include <map>
#include <stack>
#include <iostream>
#include <stdlib.h>

using namespace std;


// context consist of string name and boolean value
typedef map<string, bool> con_map;
// operator consist of char name and integer precedence
typedef map<char, int> opt_map;


// initialize expression, contexts, and operators
void initialize(string& _exp, con_map& _con, opt_map& _opt)
{
    _exp = "(\"A\"&\"B\")^(\"C\"|~\"D\")";
    cin >> _exp;
    _con["A"] = true;
    _con["B"] = false;
    _con["C"] = true;
    _con["D"] = false;
    _opt['('] = 0;
    _opt[')'] = 0;
    _opt['~'] = 3;
    _opt['&'] = 2;
    _opt['|'] = 2;
    _opt['^'] = 1;
}

// print out error message and exit program
void handle_error(const char* _msg)
{
    cout << "error: " << _msg << endl;
    exit(0);
}

// move forward the iterator and return the variable string without " symbol
// promp out notice and exit if there's error
string get_var(unsigned int& _i, string& _str)
{
    // initialize return string to be empty
    string _ret = "";
    // loop until hit another " symbol
    for(++_i; _i<_str.size(); ++_i)
    {
        if(_str[_i] == '\"')
            break;
        _ret += _str[_i];
    }
    // if there is error
    if(_i == _str.size())
        handle_error("variable not closed by \" symbol");
    // return variable string surrounded by " symbols
    return _ret;
}

// append the top item of the stack to the string and pop out that item
void stack_to_string(string& _str, stack<char>& _stk)
{
    _str += _stk.top();
    _stk.pop();
}

// get in-fix expression as input
// return post-fix expression
string shuntingYard(string& _exp, opt_map& _opt)
{
    stack<char> _opt_stack;
    string _ret = "";
    string _var = "";

    // loop each character of the expression string
    for(unsigned int _i=0; _i<_exp.size(); ++_i)
    {
        switch(_exp[_i])
        {
        // the token is a variable
        case '\"':
            // get the variable string
            _var = get_var(_i, _exp);
            // append the variable to return string
            _ret.append("\""+_var+"\"");
            break;
        // the token is a operator
        case '&':
        case '|':
        case '~':
        case '^':
            // if there's operator in stack with precedence higher than or equal to the new operator
            while(!_opt_stack.empty() && _opt[_opt_stack.top()] >= _opt[_exp[_i]])
                stack_to_string(_ret, _opt_stack);
            // push the new operator to stack anyway
            _opt_stack.push(_exp[_i]);
            break;
        // the token is a left parenthesis
        case '(':
            // push left parenthesis to stack
            _opt_stack.push(_exp[_i]);
            break;
        // the token is a right parenthesis
        case ')':
            // pop all operators between the two parenthesis to output string
            while(_opt_stack.size() && _opt_stack.top()!='(')
                stack_to_string(_ret, _opt_stack);
            // error if there's no right parenthesis left
            if(_opt_stack.empty())
                handle_error("mismatched right parenthesis");
            // pop out the right parenthesis
            _opt_stack.pop();
            break;
        }
    }

    // no more tokens to read
    while(!_opt_stack.empty())
    {
        // error if left parenthesis still exist
        if(_opt_stack.top() == '(')
            handle_error("mismatched left parenthesis");
        // pop to return string
        stack_to_string(_ret, _opt_stack);
    }

    // return the post-fix expression
    return _ret;
}

bool evaluate(string& _pf, con_map& _ct)
{
    stack<bool> _val_stack;
    bool _ret = false;
    string _var = "";
    bool _tmp1, _tmp2;

    // loop each character of the post-fix string
    for(unsigned int _i=0; _i<_pf.size(); ++_i)
    {
        switch(_pf[_i])
        {
        // the token is a variable
        case '\"':
            // get the variable string
            _var = get_var(_i, _pf);
            // push the value of the variable into stack
            _val_stack.push(_ct[_var]);
            break;
        // the token is a AND operator
        case '&':
            // error if less than two values in the stack
            if(_val_stack.size() < 2)
                handle_error("variables for AND operator mismatched");
            // pop the top two values in the stack and push in their AND value
            _tmp1 = _val_stack.top();
            _val_stack.pop();
            _tmp2 = _val_stack.top();
            _val_stack.pop();
            _val_stack.push(_tmp1&_tmp2);
            break;
        // the token is a OR operator
        case '|':
            // error if less than two values in the stack
            if(_val_stack.size() < 2)
                handle_error("variables for AND operator mismatched");
            // pop the top two values in the stack and push in their OR value
            _tmp1 = _val_stack.top();
            _val_stack.pop();
            _tmp2 = _val_stack.top();
            _val_stack.pop();
            _val_stack.push(_tmp1|_tmp2);
            break;
        // the token is a NOT operator
        case '~':
            // error if less than one values in the stack
            if(_val_stack.size() < 1)
                handle_error("variables for AND operator mismatched");
            // pop the top value in the stack and push in its NOT value
            _tmp1 = _val_stack.top();
            _val_stack.pop();
            _val_stack.push(!_tmp1);
            break;
        // the token is a XOR operator
        case '^':
            // error if less than two values in the stack
            if(_val_stack.size() < 2)
                handle_error("variables for AND operator mismatched");
            // pop the top two values in the stack and push in their XOR value
            _tmp1 = _val_stack.top();
            _val_stack.pop();
            _tmp2 = _val_stack.top();
            _val_stack.pop();
            _val_stack.push(_tmp1^_tmp2);
            break;
        }
    }

    // should be only one value left in the stack
    if(_val_stack.size() != 1)
        handle_error("operators and variables in post-fix string mismatched");

    // get teh evaluated value of the post-fix string
    _ret = _val_stack.top();

    return _ret;
}

int main()
{
    string _expression;
    con_map _context;
    opt_map _operator;

    initialize(_expression, _context, _operator);

    cout << "expression: " << _expression << endl;
    cout << "context:" << endl;
    for(con_map::iterator _i=_context.begin(); _i!=_context.end(); ++_i)
        cout << "  " << _i->first << " : " << _i->second << endl;

    string _post_fix = shuntingYard(_expression, _operator);
    cout << "post-fix: " << _post_fix <<endl;

    bool _result = evaluate(_post_fix, _context);
    cout << "result: " << _result << endl;

    return 0;
}
