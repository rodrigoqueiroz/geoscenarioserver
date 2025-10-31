# Generated from geoscenarioserver/sp/btree/parser/BTreeDSL.g4 by ANTLR 4.13.2
# encoding: utf-8
from antlr4 import *
from io import StringIO
import sys
if sys.version_info[1] > 5:
	from typing import TextIO
else:
	from typing.io import TextIO

def serializedATN():
    return [
        4,1,17,186,2,0,7,0,2,1,7,1,2,2,7,2,2,3,7,3,2,4,7,4,2,5,7,5,2,6,7,
        6,2,7,7,7,2,8,7,8,2,9,7,9,2,10,7,10,2,11,7,11,2,12,7,12,2,13,7,13,
        2,14,7,14,2,15,7,15,2,16,7,16,2,17,7,17,1,0,1,0,1,0,1,0,1,0,1,0,
        3,0,43,8,0,1,0,3,0,46,8,0,4,0,48,8,0,11,0,12,0,49,1,0,1,0,1,1,1,
        1,1,2,1,2,3,2,58,8,2,1,3,1,3,3,3,62,8,3,1,3,1,3,4,3,66,8,3,11,3,
        12,3,67,1,3,1,3,1,4,1,4,1,4,3,4,75,8,4,1,4,1,4,1,5,1,5,1,5,1,5,1,
        5,1,5,1,6,1,6,1,6,1,6,1,6,1,6,1,7,1,7,1,7,1,7,1,7,1,7,5,7,97,8,7,
        10,7,12,7,100,9,7,3,7,102,8,7,1,7,1,7,1,8,1,8,1,8,1,8,1,9,1,9,1,
        9,5,9,113,8,9,10,9,12,9,116,9,9,1,9,1,9,1,10,1,10,1,10,5,10,123,
        8,10,10,10,12,10,126,9,10,1,10,1,10,1,11,1,11,1,12,1,12,1,12,5,12,
        135,8,12,10,12,12,12,138,9,12,1,13,1,13,1,13,1,13,1,14,1,14,1,14,
        1,14,3,14,148,8,14,1,15,1,15,1,15,1,15,1,15,5,15,155,8,15,10,15,
        12,15,158,9,15,1,15,1,15,1,16,1,16,1,16,1,16,5,16,166,8,16,10,16,
        12,16,169,9,16,1,16,1,16,1,17,5,17,174,8,17,10,17,12,17,177,9,17,
        1,17,1,17,5,17,181,8,17,10,17,12,17,184,9,17,1,17,0,0,18,0,2,4,6,
        8,10,12,14,16,18,20,22,24,26,28,30,32,34,0,1,1,0,10,11,187,0,47,
        1,0,0,0,2,53,1,0,0,0,4,57,1,0,0,0,6,59,1,0,0,0,8,74,1,0,0,0,10,78,
        1,0,0,0,12,84,1,0,0,0,14,90,1,0,0,0,16,105,1,0,0,0,18,109,1,0,0,
        0,20,119,1,0,0,0,22,129,1,0,0,0,24,131,1,0,0,0,26,139,1,0,0,0,28,
        147,1,0,0,0,30,149,1,0,0,0,32,161,1,0,0,0,34,175,1,0,0,0,36,37,5,
        1,0,0,37,38,3,34,17,0,38,39,5,2,0,0,39,40,5,16,0,0,40,42,3,2,1,0,
        41,43,5,15,0,0,42,41,1,0,0,0,42,43,1,0,0,0,43,45,1,0,0,0,44,46,5,
        17,0,0,45,44,1,0,0,0,45,46,1,0,0,0,46,48,1,0,0,0,47,36,1,0,0,0,48,
        49,1,0,0,0,49,47,1,0,0,0,49,50,1,0,0,0,50,51,1,0,0,0,51,52,5,0,0,
        1,52,1,1,0,0,0,53,54,3,4,2,0,54,3,1,0,0,0,55,58,3,8,4,0,56,58,3,
        6,3,0,57,55,1,0,0,0,57,56,1,0,0,0,58,5,1,0,0,0,59,61,5,9,0,0,60,
        62,3,34,17,0,61,60,1,0,0,0,61,62,1,0,0,0,62,63,1,0,0,0,63,65,5,16,
        0,0,64,66,3,4,2,0,65,64,1,0,0,0,66,67,1,0,0,0,67,65,1,0,0,0,67,68,
        1,0,0,0,68,69,1,0,0,0,69,70,5,17,0,0,70,7,1,0,0,0,71,75,3,12,6,0,
        72,75,3,10,5,0,73,75,3,14,7,0,74,71,1,0,0,0,74,72,1,0,0,0,74,73,
        1,0,0,0,75,76,1,0,0,0,76,77,5,15,0,0,77,9,1,0,0,0,78,79,5,3,0,0,
        79,80,3,34,17,0,80,81,5,4,0,0,81,82,3,20,10,0,82,83,5,5,0,0,83,11,
        1,0,0,0,84,85,5,6,0,0,85,86,3,34,17,0,86,87,5,4,0,0,87,88,3,18,9,
        0,88,89,5,5,0,0,89,13,1,0,0,0,90,91,5,7,0,0,91,92,3,34,17,0,92,101,
        5,4,0,0,93,98,3,16,8,0,94,95,5,8,0,0,95,97,3,16,8,0,96,94,1,0,0,
        0,97,100,1,0,0,0,98,96,1,0,0,0,98,99,1,0,0,0,99,102,1,0,0,0,100,
        98,1,0,0,0,101,93,1,0,0,0,101,102,1,0,0,0,102,103,1,0,0,0,103,104,
        5,5,0,0,104,15,1,0,0,0,105,106,3,22,11,0,106,107,5,11,0,0,107,108,
        3,18,9,0,108,17,1,0,0,0,109,110,3,34,17,0,110,114,5,4,0,0,111,113,
        3,24,12,0,112,111,1,0,0,0,113,116,1,0,0,0,114,112,1,0,0,0,114,115,
        1,0,0,0,115,117,1,0,0,0,116,114,1,0,0,0,117,118,5,5,0,0,118,19,1,
        0,0,0,119,120,3,34,17,0,120,124,5,4,0,0,121,123,3,24,12,0,122,121,
        1,0,0,0,123,126,1,0,0,0,124,122,1,0,0,0,124,125,1,0,0,0,125,127,
        1,0,0,0,126,124,1,0,0,0,127,128,5,5,0,0,128,21,1,0,0,0,129,130,3,
        34,17,0,130,23,1,0,0,0,131,136,3,26,13,0,132,133,5,8,0,0,133,135,
        3,26,13,0,134,132,1,0,0,0,135,138,1,0,0,0,136,134,1,0,0,0,136,137,
        1,0,0,0,137,25,1,0,0,0,138,136,1,0,0,0,139,140,3,34,17,0,140,141,
        7,0,0,0,141,142,3,28,14,0,142,27,1,0,0,0,143,148,5,12,0,0,144,148,
        3,34,17,0,145,148,3,30,15,0,146,148,3,32,16,0,147,143,1,0,0,0,147,
        144,1,0,0,0,147,145,1,0,0,0,147,146,1,0,0,0,148,29,1,0,0,0,149,150,
        3,34,17,0,150,151,5,4,0,0,151,156,5,12,0,0,152,153,5,8,0,0,153,155,
        5,12,0,0,154,152,1,0,0,0,155,158,1,0,0,0,156,154,1,0,0,0,156,157,
        1,0,0,0,157,159,1,0,0,0,158,156,1,0,0,0,159,160,5,5,0,0,160,31,1,
        0,0,0,161,162,5,4,0,0,162,167,5,12,0,0,163,164,5,8,0,0,164,166,5,
        12,0,0,165,163,1,0,0,0,166,169,1,0,0,0,167,165,1,0,0,0,167,168,1,
        0,0,0,168,170,1,0,0,0,169,167,1,0,0,0,170,171,5,5,0,0,171,33,1,0,
        0,0,172,174,5,14,0,0,173,172,1,0,0,0,174,177,1,0,0,0,175,173,1,0,
        0,0,175,176,1,0,0,0,176,178,1,0,0,0,177,175,1,0,0,0,178,182,5,13,
        0,0,179,181,5,14,0,0,180,179,1,0,0,0,181,184,1,0,0,0,182,180,1,0,
        0,0,182,183,1,0,0,0,183,35,1,0,0,0,184,182,1,0,0,0,17,42,45,49,57,
        61,67,74,98,101,114,124,136,147,156,167,175,182
    ]

class BTreeDSLParser ( Parser ):

    grammarFileName = "BTreeDSL.g4"

    atn = ATNDeserializer().deserialize(serializedATN())

    decisionsToDFA = [ DFA(ds, i) for i, ds in enumerate(atn.decisionToState) ]

    sharedContextCache = PredictionContextCache()

    literalNames = [ "<INVALID>", "'behaviortree'", "':'", "'condition'", 
                     "'('", "')'", "'maneuver'", "'subtree'", "','", "<INVALID>", 
                     "<INVALID>", "'='" ]

    symbolicNames = [ "<INVALID>", "<INVALID>", "<INVALID>", "<INVALID>", 
                      "<INVALID>", "<INVALID>", "<INVALID>", "<INVALID>", 
                      "<INVALID>", "OPERATOR", "BOP", "ATT", "FLOAT", "WORD", 
                      "WS", "NL", "INDENT", "DEDENT" ]

    RULE_behaviorTree = 0
    RULE_rootNode = 1
    RULE_node = 2
    RULE_nodeComposition = 3
    RULE_leafNode = 4
    RULE_condition = 5
    RULE_maneuver = 6
    RULE_subtree = 7
    RULE_midconf = 8
    RULE_mconfig = 9
    RULE_cconfig = 10
    RULE_mid = 11
    RULE_params = 12
    RULE_bexpr = 13
    RULE_value = 14
    RULE_func = 15
    RULE_tupl = 16
    RULE_name = 17

    ruleNames =  [ "behaviorTree", "rootNode", "node", "nodeComposition", 
                   "leafNode", "condition", "maneuver", "subtree", "midconf", 
                   "mconfig", "cconfig", "mid", "params", "bexpr", "value", 
                   "func", "tupl", "name" ]

    EOF = Token.EOF
    T__0=1
    T__1=2
    T__2=3
    T__3=4
    T__4=5
    T__5=6
    T__6=7
    T__7=8
    OPERATOR=9
    BOP=10
    ATT=11
    FLOAT=12
    WORD=13
    WS=14
    NL=15
    INDENT=16
    DEDENT=17

    def __init__(self, input:TokenStream, output:TextIO = sys.stdout):
        super().__init__(input, output)
        self.checkVersion("4.13.2")
        self._interp = ParserATNSimulator(self, self.atn, self.decisionsToDFA, self.sharedContextCache)
        self._predicates = None




    class BehaviorTreeContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def EOF(self):
            return self.getToken(BTreeDSLParser.EOF, 0)

        def name(self, i:int=None):
            if i is None:
                return self.getTypedRuleContexts(BTreeDSLParser.NameContext)
            else:
                return self.getTypedRuleContext(BTreeDSLParser.NameContext,i)


        def INDENT(self, i:int=None):
            if i is None:
                return self.getTokens(BTreeDSLParser.INDENT)
            else:
                return self.getToken(BTreeDSLParser.INDENT, i)

        def rootNode(self, i:int=None):
            if i is None:
                return self.getTypedRuleContexts(BTreeDSLParser.RootNodeContext)
            else:
                return self.getTypedRuleContext(BTreeDSLParser.RootNodeContext,i)


        def NL(self, i:int=None):
            if i is None:
                return self.getTokens(BTreeDSLParser.NL)
            else:
                return self.getToken(BTreeDSLParser.NL, i)

        def DEDENT(self, i:int=None):
            if i is None:
                return self.getTokens(BTreeDSLParser.DEDENT)
            else:
                return self.getToken(BTreeDSLParser.DEDENT, i)

        def getRuleIndex(self):
            return BTreeDSLParser.RULE_behaviorTree

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterBehaviorTree" ):
                listener.enterBehaviorTree(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitBehaviorTree" ):
                listener.exitBehaviorTree(self)




    def behaviorTree(self):

        localctx = BTreeDSLParser.BehaviorTreeContext(self, self._ctx, self.state)
        self.enterRule(localctx, 0, self.RULE_behaviorTree)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 47 
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            while True:
                self.state = 36
                self.match(BTreeDSLParser.T__0)
                self.state = 37
                self.name()
                self.state = 38
                self.match(BTreeDSLParser.T__1)
                self.state = 39
                self.match(BTreeDSLParser.INDENT)
                self.state = 40
                self.rootNode()
                self.state = 42
                self._errHandler.sync(self)
                _la = self._input.LA(1)
                if _la==15:
                    self.state = 41
                    self.match(BTreeDSLParser.NL)


                self.state = 45
                self._errHandler.sync(self)
                _la = self._input.LA(1)
                if _la==17:
                    self.state = 44
                    self.match(BTreeDSLParser.DEDENT)


                self.state = 49 
                self._errHandler.sync(self)
                _la = self._input.LA(1)
                if not (_la==1):
                    break

            self.state = 51
            self.match(BTreeDSLParser.EOF)
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class RootNodeContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def node(self):
            return self.getTypedRuleContext(BTreeDSLParser.NodeContext,0)


        def getRuleIndex(self):
            return BTreeDSLParser.RULE_rootNode

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterRootNode" ):
                listener.enterRootNode(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitRootNode" ):
                listener.exitRootNode(self)




    def rootNode(self):

        localctx = BTreeDSLParser.RootNodeContext(self, self._ctx, self.state)
        self.enterRule(localctx, 2, self.RULE_rootNode)
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 53
            self.node()
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class NodeContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def leafNode(self):
            return self.getTypedRuleContext(BTreeDSLParser.LeafNodeContext,0)


        def nodeComposition(self):
            return self.getTypedRuleContext(BTreeDSLParser.NodeCompositionContext,0)


        def getRuleIndex(self):
            return BTreeDSLParser.RULE_node

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterNode" ):
                listener.enterNode(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitNode" ):
                listener.exitNode(self)




    def node(self):

        localctx = BTreeDSLParser.NodeContext(self, self._ctx, self.state)
        self.enterRule(localctx, 4, self.RULE_node)
        try:
            self.state = 57
            self._errHandler.sync(self)
            token = self._input.LA(1)
            if token in [3, 6, 7]:
                self.enterOuterAlt(localctx, 1)
                self.state = 55
                self.leafNode()
                pass
            elif token in [9]:
                self.enterOuterAlt(localctx, 2)
                self.state = 56
                self.nodeComposition()
                pass
            else:
                raise NoViableAltException(self)

        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class NodeCompositionContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def OPERATOR(self):
            return self.getToken(BTreeDSLParser.OPERATOR, 0)

        def INDENT(self):
            return self.getToken(BTreeDSLParser.INDENT, 0)

        def DEDENT(self):
            return self.getToken(BTreeDSLParser.DEDENT, 0)

        def name(self):
            return self.getTypedRuleContext(BTreeDSLParser.NameContext,0)


        def node(self, i:int=None):
            if i is None:
                return self.getTypedRuleContexts(BTreeDSLParser.NodeContext)
            else:
                return self.getTypedRuleContext(BTreeDSLParser.NodeContext,i)


        def getRuleIndex(self):
            return BTreeDSLParser.RULE_nodeComposition

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterNodeComposition" ):
                listener.enterNodeComposition(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitNodeComposition" ):
                listener.exitNodeComposition(self)




    def nodeComposition(self):

        localctx = BTreeDSLParser.NodeCompositionContext(self, self._ctx, self.state)
        self.enterRule(localctx, 6, self.RULE_nodeComposition)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 59
            self.match(BTreeDSLParser.OPERATOR)
            self.state = 61
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            if _la==13 or _la==14:
                self.state = 60
                self.name()


            self.state = 63
            self.match(BTreeDSLParser.INDENT)
            self.state = 65 
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            while True:
                self.state = 64
                self.node()
                self.state = 67 
                self._errHandler.sync(self)
                _la = self._input.LA(1)
                if not ((((_la) & ~0x3f) == 0 and ((1 << _la) & 712) != 0)):
                    break

            self.state = 69
            self.match(BTreeDSLParser.DEDENT)
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class LeafNodeContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def NL(self):
            return self.getToken(BTreeDSLParser.NL, 0)

        def maneuver(self):
            return self.getTypedRuleContext(BTreeDSLParser.ManeuverContext,0)


        def condition(self):
            return self.getTypedRuleContext(BTreeDSLParser.ConditionContext,0)


        def subtree(self):
            return self.getTypedRuleContext(BTreeDSLParser.SubtreeContext,0)


        def getRuleIndex(self):
            return BTreeDSLParser.RULE_leafNode

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterLeafNode" ):
                listener.enterLeafNode(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitLeafNode" ):
                listener.exitLeafNode(self)




    def leafNode(self):

        localctx = BTreeDSLParser.LeafNodeContext(self, self._ctx, self.state)
        self.enterRule(localctx, 8, self.RULE_leafNode)
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 74
            self._errHandler.sync(self)
            token = self._input.LA(1)
            if token in [6]:
                self.state = 71
                self.maneuver()
                pass
            elif token in [3]:
                self.state = 72
                self.condition()
                pass
            elif token in [7]:
                self.state = 73
                self.subtree()
                pass
            else:
                raise NoViableAltException(self)

            self.state = 76
            self.match(BTreeDSLParser.NL)
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class ConditionContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def name(self):
            return self.getTypedRuleContext(BTreeDSLParser.NameContext,0)


        def cconfig(self):
            return self.getTypedRuleContext(BTreeDSLParser.CconfigContext,0)


        def getRuleIndex(self):
            return BTreeDSLParser.RULE_condition

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterCondition" ):
                listener.enterCondition(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitCondition" ):
                listener.exitCondition(self)




    def condition(self):

        localctx = BTreeDSLParser.ConditionContext(self, self._ctx, self.state)
        self.enterRule(localctx, 10, self.RULE_condition)
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 78
            self.match(BTreeDSLParser.T__2)
            self.state = 79
            self.name()
            self.state = 80
            self.match(BTreeDSLParser.T__3)
            self.state = 81
            self.cconfig()
            self.state = 82
            self.match(BTreeDSLParser.T__4)
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class ManeuverContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def name(self):
            return self.getTypedRuleContext(BTreeDSLParser.NameContext,0)


        def mconfig(self):
            return self.getTypedRuleContext(BTreeDSLParser.MconfigContext,0)


        def getRuleIndex(self):
            return BTreeDSLParser.RULE_maneuver

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterManeuver" ):
                listener.enterManeuver(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitManeuver" ):
                listener.exitManeuver(self)




    def maneuver(self):

        localctx = BTreeDSLParser.ManeuverContext(self, self._ctx, self.state)
        self.enterRule(localctx, 12, self.RULE_maneuver)
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 84
            self.match(BTreeDSLParser.T__5)
            self.state = 85
            self.name()
            self.state = 86
            self.match(BTreeDSLParser.T__3)
            self.state = 87
            self.mconfig()
            self.state = 88
            self.match(BTreeDSLParser.T__4)
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class SubtreeContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def name(self):
            return self.getTypedRuleContext(BTreeDSLParser.NameContext,0)


        def midconf(self, i:int=None):
            if i is None:
                return self.getTypedRuleContexts(BTreeDSLParser.MidconfContext)
            else:
                return self.getTypedRuleContext(BTreeDSLParser.MidconfContext,i)


        def getRuleIndex(self):
            return BTreeDSLParser.RULE_subtree

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterSubtree" ):
                listener.enterSubtree(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitSubtree" ):
                listener.exitSubtree(self)




    def subtree(self):

        localctx = BTreeDSLParser.SubtreeContext(self, self._ctx, self.state)
        self.enterRule(localctx, 14, self.RULE_subtree)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 90
            self.match(BTreeDSLParser.T__6)
            self.state = 91
            self.name()
            self.state = 92
            self.match(BTreeDSLParser.T__3)
            self.state = 101
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            if _la==13 or _la==14:
                self.state = 93
                self.midconf()
                self.state = 98
                self._errHandler.sync(self)
                _la = self._input.LA(1)
                while _la==8:
                    self.state = 94
                    self.match(BTreeDSLParser.T__7)
                    self.state = 95
                    self.midconf()
                    self.state = 100
                    self._errHandler.sync(self)
                    _la = self._input.LA(1)



            self.state = 103
            self.match(BTreeDSLParser.T__4)
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class MidconfContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def mid(self):
            return self.getTypedRuleContext(BTreeDSLParser.MidContext,0)


        def ATT(self):
            return self.getToken(BTreeDSLParser.ATT, 0)

        def mconfig(self):
            return self.getTypedRuleContext(BTreeDSLParser.MconfigContext,0)


        def getRuleIndex(self):
            return BTreeDSLParser.RULE_midconf

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterMidconf" ):
                listener.enterMidconf(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitMidconf" ):
                listener.exitMidconf(self)




    def midconf(self):

        localctx = BTreeDSLParser.MidconfContext(self, self._ctx, self.state)
        self.enterRule(localctx, 16, self.RULE_midconf)
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 105
            self.mid()
            self.state = 106
            self.match(BTreeDSLParser.ATT)
            self.state = 107
            self.mconfig()
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class MconfigContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def name(self):
            return self.getTypedRuleContext(BTreeDSLParser.NameContext,0)


        def params(self, i:int=None):
            if i is None:
                return self.getTypedRuleContexts(BTreeDSLParser.ParamsContext)
            else:
                return self.getTypedRuleContext(BTreeDSLParser.ParamsContext,i)


        def getRuleIndex(self):
            return BTreeDSLParser.RULE_mconfig

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterMconfig" ):
                listener.enterMconfig(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitMconfig" ):
                listener.exitMconfig(self)




    def mconfig(self):

        localctx = BTreeDSLParser.MconfigContext(self, self._ctx, self.state)
        self.enterRule(localctx, 18, self.RULE_mconfig)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 109
            self.name()
            self.state = 110
            self.match(BTreeDSLParser.T__3)
            self.state = 114
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            while _la==13 or _la==14:
                self.state = 111
                self.params()
                self.state = 116
                self._errHandler.sync(self)
                _la = self._input.LA(1)

            self.state = 117
            self.match(BTreeDSLParser.T__4)
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class CconfigContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def name(self):
            return self.getTypedRuleContext(BTreeDSLParser.NameContext,0)


        def params(self, i:int=None):
            if i is None:
                return self.getTypedRuleContexts(BTreeDSLParser.ParamsContext)
            else:
                return self.getTypedRuleContext(BTreeDSLParser.ParamsContext,i)


        def getRuleIndex(self):
            return BTreeDSLParser.RULE_cconfig

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterCconfig" ):
                listener.enterCconfig(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitCconfig" ):
                listener.exitCconfig(self)




    def cconfig(self):

        localctx = BTreeDSLParser.CconfigContext(self, self._ctx, self.state)
        self.enterRule(localctx, 20, self.RULE_cconfig)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 119
            self.name()
            self.state = 120
            self.match(BTreeDSLParser.T__3)
            self.state = 124
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            while _la==13 or _la==14:
                self.state = 121
                self.params()
                self.state = 126
                self._errHandler.sync(self)
                _la = self._input.LA(1)

            self.state = 127
            self.match(BTreeDSLParser.T__4)
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class MidContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def name(self):
            return self.getTypedRuleContext(BTreeDSLParser.NameContext,0)


        def getRuleIndex(self):
            return BTreeDSLParser.RULE_mid

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterMid" ):
                listener.enterMid(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitMid" ):
                listener.exitMid(self)




    def mid(self):

        localctx = BTreeDSLParser.MidContext(self, self._ctx, self.state)
        self.enterRule(localctx, 22, self.RULE_mid)
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 129
            self.name()
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class ParamsContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def bexpr(self, i:int=None):
            if i is None:
                return self.getTypedRuleContexts(BTreeDSLParser.BexprContext)
            else:
                return self.getTypedRuleContext(BTreeDSLParser.BexprContext,i)


        def getRuleIndex(self):
            return BTreeDSLParser.RULE_params

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterParams" ):
                listener.enterParams(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitParams" ):
                listener.exitParams(self)




    def params(self):

        localctx = BTreeDSLParser.ParamsContext(self, self._ctx, self.state)
        self.enterRule(localctx, 24, self.RULE_params)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 131
            self.bexpr()
            self.state = 136
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            while _la==8:
                self.state = 132
                self.match(BTreeDSLParser.T__7)
                self.state = 133
                self.bexpr()
                self.state = 138
                self._errHandler.sync(self)
                _la = self._input.LA(1)

        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class BexprContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def name(self):
            return self.getTypedRuleContext(BTreeDSLParser.NameContext,0)


        def value(self):
            return self.getTypedRuleContext(BTreeDSLParser.ValueContext,0)


        def BOP(self):
            return self.getToken(BTreeDSLParser.BOP, 0)

        def ATT(self):
            return self.getToken(BTreeDSLParser.ATT, 0)

        def getRuleIndex(self):
            return BTreeDSLParser.RULE_bexpr

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterBexpr" ):
                listener.enterBexpr(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitBexpr" ):
                listener.exitBexpr(self)




    def bexpr(self):

        localctx = BTreeDSLParser.BexprContext(self, self._ctx, self.state)
        self.enterRule(localctx, 26, self.RULE_bexpr)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 139
            self.name()
            self.state = 140
            _la = self._input.LA(1)
            if not(_la==10 or _la==11):
                self._errHandler.recoverInline(self)
            else:
                self._errHandler.reportMatch(self)
                self.consume()
            self.state = 141
            self.value()
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class ValueContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def FLOAT(self):
            return self.getToken(BTreeDSLParser.FLOAT, 0)

        def name(self):
            return self.getTypedRuleContext(BTreeDSLParser.NameContext,0)


        def func(self):
            return self.getTypedRuleContext(BTreeDSLParser.FuncContext,0)


        def tupl(self):
            return self.getTypedRuleContext(BTreeDSLParser.TuplContext,0)


        def getRuleIndex(self):
            return BTreeDSLParser.RULE_value

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterValue" ):
                listener.enterValue(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitValue" ):
                listener.exitValue(self)




    def value(self):

        localctx = BTreeDSLParser.ValueContext(self, self._ctx, self.state)
        self.enterRule(localctx, 28, self.RULE_value)
        try:
            self.state = 147
            self._errHandler.sync(self)
            la_ = self._interp.adaptivePredict(self._input,12,self._ctx)
            if la_ == 1:
                self.enterOuterAlt(localctx, 1)
                self.state = 143
                self.match(BTreeDSLParser.FLOAT)
                pass

            elif la_ == 2:
                self.enterOuterAlt(localctx, 2)
                self.state = 144
                self.name()
                pass

            elif la_ == 3:
                self.enterOuterAlt(localctx, 3)
                self.state = 145
                self.func()
                pass

            elif la_ == 4:
                self.enterOuterAlt(localctx, 4)
                self.state = 146
                self.tupl()
                pass


        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class FuncContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def name(self):
            return self.getTypedRuleContext(BTreeDSLParser.NameContext,0)


        def FLOAT(self, i:int=None):
            if i is None:
                return self.getTokens(BTreeDSLParser.FLOAT)
            else:
                return self.getToken(BTreeDSLParser.FLOAT, i)

        def getRuleIndex(self):
            return BTreeDSLParser.RULE_func

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterFunc" ):
                listener.enterFunc(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitFunc" ):
                listener.exitFunc(self)




    def func(self):

        localctx = BTreeDSLParser.FuncContext(self, self._ctx, self.state)
        self.enterRule(localctx, 30, self.RULE_func)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 149
            self.name()
            self.state = 150
            self.match(BTreeDSLParser.T__3)
            self.state = 151
            self.match(BTreeDSLParser.FLOAT)
            self.state = 156
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            while _la==8:
                self.state = 152
                self.match(BTreeDSLParser.T__7)
                self.state = 153
                self.match(BTreeDSLParser.FLOAT)
                self.state = 158
                self._errHandler.sync(self)
                _la = self._input.LA(1)

            self.state = 159
            self.match(BTreeDSLParser.T__4)
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class TuplContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def FLOAT(self, i:int=None):
            if i is None:
                return self.getTokens(BTreeDSLParser.FLOAT)
            else:
                return self.getToken(BTreeDSLParser.FLOAT, i)

        def getRuleIndex(self):
            return BTreeDSLParser.RULE_tupl

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterTupl" ):
                listener.enterTupl(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitTupl" ):
                listener.exitTupl(self)




    def tupl(self):

        localctx = BTreeDSLParser.TuplContext(self, self._ctx, self.state)
        self.enterRule(localctx, 32, self.RULE_tupl)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 161
            self.match(BTreeDSLParser.T__3)
            self.state = 162
            self.match(BTreeDSLParser.FLOAT)
            self.state = 167
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            while _la==8:
                self.state = 163
                self.match(BTreeDSLParser.T__7)
                self.state = 164
                self.match(BTreeDSLParser.FLOAT)
                self.state = 169
                self._errHandler.sync(self)
                _la = self._input.LA(1)

            self.state = 170
            self.match(BTreeDSLParser.T__4)
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class NameContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def WORD(self):
            return self.getToken(BTreeDSLParser.WORD, 0)

        def WS(self, i:int=None):
            if i is None:
                return self.getTokens(BTreeDSLParser.WS)
            else:
                return self.getToken(BTreeDSLParser.WS, i)

        def getRuleIndex(self):
            return BTreeDSLParser.RULE_name

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterName" ):
                listener.enterName(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitName" ):
                listener.exitName(self)




    def name(self):

        localctx = BTreeDSLParser.NameContext(self, self._ctx, self.state)
        self.enterRule(localctx, 34, self.RULE_name)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 175
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            while _la==14:
                self.state = 172
                self.match(BTreeDSLParser.WS)
                self.state = 177
                self._errHandler.sync(self)
                _la = self._input.LA(1)

            self.state = 178
            self.match(BTreeDSLParser.WORD)
            self.state = 182
            self._errHandler.sync(self)
            _alt = self._interp.adaptivePredict(self._input,16,self._ctx)
            while _alt!=2 and _alt!=ATN.INVALID_ALT_NUMBER:
                if _alt==1:
                    self.state = 179
                    self.match(BTreeDSLParser.WS) 
                self.state = 184
                self._errHandler.sync(self)
                _alt = self._interp.adaptivePredict(self._input,16,self._ctx)

        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx





