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
behavior_tree       : ('BehaviorTree' name ':' INDENT
                       root_node NL? DEDENT?)+ EOF;

root_node           : node;
node                : leaf_node | node_composition;
node_composition    : OPERATOR INDENT node+ DEDENT;
leaf_node           : (maneuver | condition | subtree) NL;

condition           : 'Condition' name ('(' params ')')?;
maneuver            : 'Maneuver' name ('(' key (',' params)* ')')?;
subtree             : 'Subtree' name ('(' params ')')?;

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
WS                  : (' ' | '\t') ;
/*
 * From antlr-denter to help with IDENT and DEDENT tokens
 */
NL: ('\r'? '\n' ' '*); //For tabs just switch out ' '* with '\t'*