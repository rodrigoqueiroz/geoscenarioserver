//dinizr@chalmers.se
//to generate python parser: antlr4 -Dlanguage=Python3 BTreeDSL.g4
grammar BTreeDSL;

tokens { INDENT, DEDENT }

@lexer::header{
from antlr_denter.DenterHelper import DenterHelper
from BTreeDSLParser import BTreeDSLParser
}
@lexer::members {
class BTreeDSLDenter(DenterHelper):
    def __init__(self, lexer, nl_token, indent_token, dedent_token, ignore_eof):
        super().__init__(nl_token, indent_token, dedent_token, ignore_eof)
        self.lexer: BTreeDSLLexer = lexer

    def pull_token(self):
        return super(BTreeDSLLexer, self.lexer).nextToken()

denter = None

def nextToken(self):
    if not self.denter:
        self.denter = self.BTreeDSLDenter(self, self.NL, BTreeDSLParser.INDENT, BTreeDSLParser.DEDENT, ignore_eof=False)
    return self.denter.next_token()

}

/*
* Parser Rules
*/
behaviorTree        : ('BehaviorTree' name ':' INDENT rootNode NL? DEDENT?)+ EOF;

rootNode            : node;
node                : leafNode | nodeComposition;
nodeComposition     : OPERATOR INDENT node+ DEDENT;
leafNode            : (maneuver | condition | subtree) NL;

condition           : 'Condition' name ('('  key (',' params)* ')');
maneuver            : 'Maneuver' name ('(' key (',' params)* ')');
subtree             : 'Subtree' name ('(' params* ')')?;

key                 : 'key' ATT name ;
params              : bexpr (',' bexpr)* ;
bexpr               : name (BOP|ATT) value ;
value               : FLOAT | name;
name                : WS* WORD WS*;

/*
* Lexer Rules
*/
OPERATOR            : '?' | '->' | '||';
BOP                 : '<'|'>'|'=='|'>='|'<='|'!=';
ATT                 : '=';
FLOAT               : [+-]?([0-9]*[.])?[0-9]+;
WORD                : ([a-z] | [A-Z] | '_')+ ;
WS                  : (' ' | '\t') -> skip;
/*
 * From antlr-denter to help with INDENT and DEDENT tokens
 */
NL: ('\r'? '\n' ' '*); //For tabs just switch out ' '* with '\t'*