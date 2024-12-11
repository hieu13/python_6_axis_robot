def PLC_error_write(self, status):
    match status:
        case 1:
            return "(1)コンベアリール確認失敗"
        case 2:
            return "(2)コンベアリール押さえ出し失敗"
        case 3:
            return "(3)一本送り爪前後出し失敗"
        case 4:
            return "(4)一本送り送り出し失敗"
        case 5:
            return "(5)一本送り少し戻し失敗"
        case 6:
            return "(6)一本送り爪前後戻し失敗"
        case 7:
            return "(7)一本送り戻し失敗"
        case 8:
            return "(8)コンベアリール押さえ戻し失敗"
        case 9:
            return "(9)表裏位置リール上押さえ出失敗"
        case 10:
            return "(10)表裏位置回転上昇失敗"
        case 11:
            return "(11)表裏位置リール到着確認失敗"
        case 12:
            return "(12)表裏位置決め高速失敗"
        case 13:
            return "(13)表裏位置決め低速失敗"
        case 14:
            return "(14)リール表裏確認 NG"
        case 15:
            return "(15)表裏確認通信エラー"
        case 16:
            return "(16)持ち上げ爪初期位置待機失敗"
        case 17:
            return "(17)リール持ち上げ爪出失敗"
        case 18:
            return "(18)表裏回転部上下下降失敗"
        case 19:
            return "(19)表裏位置上押さえ戻失敗"
        case 20:
            return "(20)ラベル位置上押さえ出失敗"
        case 21:
            return "(21)持ち上げ爪ラベル位置移動失敗"
        case 22:
            return "(22)持ち上げ爪 P４へ移動NG"
        case 23:
            return "(23)持ち上げ爪 P1へ移動NG"
        case 24:
            return "(24)ラベル位置回転部前後出失敗"
        case 25:
            return "(25)持ち上げ爪少し下げる失敗"
        case 26:
            return "(26)ラベル位置リール到着確認失敗"
        case 27:
            return "(27)ラベル貼り位置決め高速失敗"
        case 28:
            return "(28)ラベル貼り位置決め低速失敗"
        case 29:
            return "(29)持ち上げ爪少し上げ失敗"
        case 30:
            return "(30)ラベル位置回転部前後戻失敗"
        case 31:
            return "(31)リール落下防止出失敗"
        case 32:
            return "(32)持ち上げ爪少し下げる2失敗"
        case 33:
            return "(33)ラベル位置リール上押さえ戻失敗"
        case 34:
            return "(34)持ち上げ爪前後戻失敗"
        case 35:
            return "(35)リール送り出し出失敗"
        case 36:
            return "(36)リール送り出し戻失敗"
        case 37:
            return "(37)リール落下防止戻失敗"
        case 38:
            return "(38)持ち上げ爪初期位置戻失敗"
        case 39:
            return "(39)リールラベル発行失敗"
        case 40:
            return "(40)リールラベル確認失敗"
        case 41:
            return "(41)リールラベル吸着上下出失敗"
        case 42:
            return "(42)リールラベル吸着失敗"
        case 43:
            return "(43)リールラベル吸着上下戻失敗"
        case 44:
            return "(44)リールラベル吸着前後戻失敗"
        case 45:
            return "(45)リールラベル吸着回転回失敗"
        case 46:
            return "(46)ラベル貼り時リール押さえ出失敗"
        case 47:
            return "(47)リールラベル吸着上下出2失敗"
        case 48:
            return "(48)リールラベル吸着解除失敗"
        case 49:
            return "(49)ラベル貼り時リール押さえ戻失敗"
        case 50:
            return "(50)リールラベル吸着上下戻2失敗"
        case 51:
            return "(51)リールラベル吸着回転戻失敗"
        case 52:
            return "(52)リールラベル吸着前後出失敗"
        case 53:
            return "(53)リール送り出し失敗"
        case 54:
            return "(54)バーコード位置到着確認失敗"
        case 55:
            return "(55)バーコード位置上押さえ出失敗"
        case 56:
            return "(56)バーコード位置上回転部上下出失敗"
        case 57:
            return "(57)バーコード位置決め高速失敗"
        case 58:
            return "(58)バーコード位置決め低速失敗"
        case 59:
            return "(59)バーコード確認失敗"
        case 60:
            return "(60)位置決め爪戻失敗"
        case 61:
            return "(61)リール押し出し出失敗"
        case 62:
            return "(62)バーコード位置上押さえ戻失敗"
        case 63:
            return "(63)バーコード位置回転部上下戻失敗"
        case 64:
            return "(64)位置決め爪出失敗"
        case 65:
            return "(65)リール押し出し戻失敗"
        case 66:
            return "(66)箱有確認失敗"
        case 67:
            return "(67)箱切れ"
        case 68:
            return "(68)平箱持ち上げ1出失敗"
        case 69:
            return "(69)平箱持ち上げ1・2戻失敗"
        case 70:
            return "(70)平箱供給確認1失敗"
        case 71:
            return "(71)平箱持ち上げ1出失敗"
        case 72:
            return "(72)平箱供給確認2失敗"
        case 73:
            return "(73)平箱持ち上げ2出失敗"
        case 74:
            return "(74)平箱到着失敗"
        case 75:
            return "(75)平箱持ち上げ1・2戻失敗"
        case 76:
            return "(76)平箱表裏判定 NG"
        case 77:
            return "(77)平箱吸着上上下出失敗"
        case 78:
            return "(78)平箱吸着下上下出失敗"
        case 79:
            return "(79)平箱吸着下失敗"
        case 80:
            return "(80)平箱吸着上失敗"
        case 81:
            return "(81)箱開フラップモータセット失敗"
        case 82:
            return "(82)箱開フラップ前後出失敗"
        case 83:
            return "(83)箱ズレ"
        case 84:
            return "(84)平箱吸着上上下戻失敗"
        case 85:
            return "(85)平箱吸着下上下戻失敗"
        case 86:
            return "(86)箱開フラップモータP2失敗"
        case 87:
            return "(87)箱開フラップモータP3失敗"
        case 88:
            return "(88)箱開フラップモータP2戻し失敗"
        case 89:
            return "(89)箱折り前後出失敗"
        case 90:
            return "(90)箱折り曲げ出失敗"
        case 91:
            return "(91)箱折り上押さえ出失敗"
        case 92:
            return "(92)箱折り上押さえ戻失敗"
        case 93:
            return "(93)箱折り上押さえ出失敗"
        case 94:
            return "(94)箱折り上押さえ戻失敗"
        case 95:
            return "(95)箱折り上押さえ出失敗"
        case 96:
            return "(96)箱折り上押さえ出失敗"
        case 97:
            return "(97)箱開フラップ前後戻失敗"
        case 98:
            return "(98)箱開フラップモータ戻失敗"
        case 99:
            return "(99)箱ラベル発行失敗"
        case 100:
            return "(100)箱ラベル確認失敗"
        case 101:
            return "(101)箱ラベル吸着上下出失敗"
        case 102:
            return "(102)箱ラベル吸着失敗"
        case 103:
            return "(103)箱ラベル吸着上下戻失敗"
        case 104:
            return "(104)箱ラベル吸着前後戻失敗"
        case 105:
            return "(105)ラベル貼り時箱押さえ出失敗"
        case 106:
            return "(106)箱ラベル吸着回転回失敗"
        case 107:
            return "(107)箱ラベル吸着上下出失敗"
        case 108:
            return "(108)箱ラベル吸着解除確認失敗"
        case 109:
            return "(109)箱ラベル吸着上下戻2失敗"
        case 110:
            return "(110)ラベル貼り時箱押さえ戻失敗"
        case 111:
            return "(111)箱ラベル吸着回転戻失敗"
        case 112:
            return "(112)箱ラベル吸着前後出失敗"
        case 113:
            return "(113)箱挿入待機失敗"
        case 114:
            return "(114)リール転がり受け戻失敗"
        case 115:
            return "(115)5本横送り送り出し失敗"
        case 116:
            return "(116)5本横送り戻失敗"
        case 117:
            return "(117)リール転がり受け出失敗"
        case 118:
            return "(118)箱挿入フラップ開失敗"
        case 119:
            return "(119)箱挿入送り出し失敗"
        case 120:
            return "(120)箱挿入戻し失敗"
        case 121:
            return "(121)箱挿入フラップ閉失敗"
        case 122:
            return "(122)箱移動1出失敗"
        case 123:
            return "(123)箱移動2出失敗"
        case 124:
            return "(124)箱回転チャック閉失敗"
        case 125:
            return "(125)箱回転チャック回失敗"
        case 126:
            return "(126)箱移動1戻失敗"
        case 127:
            return "(127)リール入り箱受け前後出失敗"
        case 128:
            return "(128)リール入り箱受け上下出失敗"
        case 129:
            return "(129)箱回転チャック開失敗"
        case 130:
            return "(130)リール入り箱受け上下戻失敗"
        case 131:
            return "(131)リール入り箱受け上下出失敗"
        case 132:
            return "(132)箱移動2戻失敗"
        case 133:
            return "(133)リール入り箱受け前後戻失敗"
        case 134:
            return "(134)リール入り箱受け上下戻失敗"
        case 135:
            return "(135)箱位置１確認失敗"
        case 136:
            return "(136)箱横移動吸着上下出失敗"
        case 137:
            return "(137)箱横移動吸着確認失敗"
        case 138:
            return "(138)箱横移動POS2(ラベル貼り部)移動失敗"
        case 139:
            return "(139)箱90度回転失敗"
        case 140:
            return "(140)箱90度戻す失敗"
        case 141:
            return "(141)箱横移動POS3(受け渡し部)移動失敗"
        case 142:
            return "(142)箱渡し失敗"
        case 143:
            return "(143)箱横移動戻り失敗"
        case 144:
            return "(144)nani"
        case 145:
            return "(145)nani"
        case 146:
            return "(146)nani"
        case 147:
            return "(147)箱満載"
        case 148:
            return "(148)5本ストックリール倒れ"
        case 149:
            return "(149)リール落下"
        case 150:
            return "(150)リール本数要確認"
        case 151:
            return "(151)リール浮かし前後出失敗"
        case 152:
            return "(152リール浮かし上下出失敗"
        case 153:
            return "(153)リール浮かし上下戻失敗"
        case 154:
            return "(154)リール浮かし前後出失敗"
        case 155:
            return "(155)表裏確認3回失敗"
        case 200:
            return "(200)リール2枚残り確認"
        case 201:
            return "(201)元圧異常"
        case 202:
            return "(202)一本送り原点復帰失敗"
        case 203:
            return "(203)一本送り爪原点失敗"
        case 204:
            return "(204)コンベアリール押さえ原点失敗"
        case 205:
            return "(205)持ち上げ爪原点失敗"
        case 206:
            return "(206)持ち上げ爪前後原点失敗"
        case 207:
            return "(207)表裏回転部上下原点失敗"
        case 208:
            return "(208)表裏回転部上押さえ原点失敗"
        case 209:
            return "(209)ラベル貼り回転前後原点失敗"
        case 210:
            return "(210)ラベル貼り上押さえ原点失敗"
        case 211:
            return "(211)ラベル貼りリール押さえ原点失敗"
        case 212:
            return "(212)リール落下防止原点失敗"
        case 213:
            return "(213)リール送り出し原点失敗"
        case 214:
            return "(214)バーコード回転上下原点失敗"
        case 215:
            return "(215)バーコード回転上押さえ原点失敗"
        case 216:
            return "(216)リール位置決め爪原点失敗"
        case 217:
            return "(217)リール押し出し原点失敗"
        case 218:
            return "(218)リール転がり受け原点失敗"
        case 219:
            return "(219)リールラベル吸着上下原点失敗"
        case 220:
            return "(220)リールラベル吸着回転原点失敗"
        case 221:
            return "(221)リールラベル吸着前後原点失敗"
        case 222:
            return "(222)平箱持ち上げ1原点失敗"
        case 223:
            return "(223)平箱持ち上げ2原点失敗"
        case 224:
            return "(224)平箱吸着下上下原点失敗"
        case 225:
            return "(225)平箱吸着上上下原点失敗"
        case 226:
            return "(226)平箱開フラップ前後原点失敗"
        case 227:
            return "(227)平箱開フラップ回転原点失敗"
        case 228:
            return "(228)箱折り上押さえ原点失敗"
        case 229:
            return "(229)箱ラベル吸着上下原点失敗"
        case 230:
            return "(230)箱ラベル吸着回転原点失敗"
        case 231:
            return "(231)箱ラベル吸着前後原点失敗"
        case 232:
            return "(232)ラベル貼り時箱押さえ原点失敗"
        case 233:
            return "(233)5本横送り原点失敗"
        case 234:
            return "(234)箱挿入原点失敗"
        case 235:
            return "(235)箱横移動上下原点失敗"
        case 236:
            return "(236)箱横移動回転原点失敗"
        case 237:
            return "(237)リール入り箱受け上下原点失敗"
        case 238:
            return "(238)リール入り箱受け前後原点失敗"
        case 239:
            return "(239)箱挿入フラップ原点失敗"
        case 240:
            return "(240)箱移動1原点失敗"
        case 241:
            return "(241)箱横移動2原点失敗"
        case 242:
            return "(242)箱チャック開閉原点失敗"
        case 243:
            return "(243)箱横移動原点復帰失敗"
        case 250:
            return "(250)平箱供給部箱残り"
        case 251:
            return "(251)原点復帰未完了"

        case 301:
            return "ロット終了"
        case 302:
            return "(302)ラベル貼り付け総数ズレ(ラベル余り)"
        case 303:
            return "(303)ラベルデータ未受信"
        case 304:
            return "(304)ラベル貼り付け総数ズレ(リール余り)"
        case 305:
            return "(305)手動モード未解除"
        case 306:
            return "(306)5本ストックリール倒れ"
        case 307:
            return "(307)リールラベル残り"
        case 308:
            return "(308)非常停止スイッチON"
        case 309:
            return "(309)ラベルなし箱落下"
        case 310:
            return "(310)ラベル付き箱落下"
        case 311:
            return "(311)復帰後箱確認失敗"
        case 312:
            return "(312)箱回転チャック回転失敗"
        case 313:
            return "(313)箱回転チャック開失敗"
        case _:
            return ""


#############################
def PLC_setume_write(self, status):
    match status:
        case 1:
            return "(1)コンベア上のリールを確認し問題がなければRESET→STARTで再開してください。"
        case 2:
            return "(2)RESET→STARTで再開してください。"
        case 3:
            return "(3)RESET→STARTで再開してください。"
        case 4:
            return "(4)詰まったリールを除去し、アームを端に戻し装置の電源を再起動してください"
        case 5:
            return "(5)詰まったリールを除去し、アームを端に戻し装置の電源を再起動してください"
        case 6:
            return "(6)詰まったリールを除去し、アームを端に戻し装置の電源を再起動してください"
        case 7:
            return "(7)詰まったリールを除去し、アームを端に戻し装置の電源を再起動してください"
        case 8:
            return "(8)RESET→STARTで再開してください。"
        case 9:
            return "(9)詰まったリールの状態を確認し、続行可能であればRESET→STARTで再開、潰れていれば該当リール、吸着中のリールラベルを取り除き、取り除いた場所に次のリールを入れ、ストック部に赤リールを入れリール本数を1増やしRESET→STARTで再開してください。"
        case 10:
            return "(10)詰まったリールの状態を確認し、続行可能であればRESET→STARTで再開、潰れていれば該当リール、吸着中のリールラベルを取り除き、取り除いた場所に次のリールを入れ、ストック部に赤リールを入れリール本数を1増やしRESET→STARTで再開してください。"
        case 11:
            return "(11)RESET→STARTで再開してください。"
        case 12:
            return "(12)排出されたリール・吸着中のリールラベルを取り除き、5本ストック部に赤リールを入れRESET→STARTで再開してください。"
        case 13:
            return "(13)排出されたリール・吸着中のリールラベルを取り除き、5本ストック部に赤リールを入れRESET→STARTで再開してください。"
        case 14:
            return "(14)排出されたリールの表裏を確認後、排出された位置に戻しRESET→STARTで再開してください。"
        case 15:
            return "(15)RESET→STARTで再開してください。"
        case 16:
            return "(16)装置の電源を再起動してください"
        case 17:
            return "(17)RESET→STARTで再開してください。"
        case 18:
            return "(18)RESET→STARTで再開してください。"
        case 19:
            return "(19)RESET→STARTで再開してください。"
        case 20:
            return "(20)RESET→STARTで再開してください。"
        case 21:
            return "(21)詰まったリールを除去し、装置の電源を再起動してください。"
        case 22:
            return "(22)詰まったリールを除去し、装置の電源を再起動してください。"
        case 23:
            return "(23)詰まったリールを除去し、装置の電源を再起動してください。"
        case 24:
            return "(24)RESET→STARTで再開してください。"
        case 25:
            return "(25)詰まったリールを除去し、装置の電源を再起動してください。"
        case 26:
            return "(26)RESET→STARTで再開してください。"
        case 27:
            return "(27)排出されたリール・吸着中のリールラベルを取り除き、5本ストック部に赤リールを入れRESET→STARTで再開してください。"
        case 28:
            return "(28)排出されたリール・吸着中のリールラベルを取り除き、5本ストック部に赤リールを入れRESET→STARTで再開してください。"
        case 29:
            return "(29)詰まったリールを除去し、装置の電源を再起動してください。"
        case 30:
            return "(30)RESET→STARTで再開してください。"
        case 31:
            return "(31)RESET→STARTで再開してください。"
        case 32:
            return "(32)装置の電源を再起動してください。"
        case 33:
            return "(33)RESET→STARTで再開してください。"
        case 34:
            return "(34)RESET→STARTで再開してください。"
        case 35:
            return "(35)RESET→STARTで再開してください。"
        case 36:
            return "(36)RESET→STARTで再開してください。"
        case 37:
            return "(37)RESET→STARTで再開してください。"
        case 38:
            return "(38)装置の電源を再起動してください。"
        case 39:
            return "(39)プリンタの状態を確認し、RESET→STARTで再開してください。"
        case 40:
            return "(40)リールラベルがプリンタ台にあることを確認し、RESET→STARTで再開してください。"
        case 41:
            return "(41)RESET→STARTで再開してください。"
        case 42:
            return "(42)リールラベルがプリンタ台にあることを確認し、RESET→STARTで再開してください。"
        case 43:
            return "(43)RESET→STARTで再開してください。"
        case 44:
            return "(44)RESET→STARTで再開してください。"
        case 45:
            return "(45)RESET→STARTで再開してください。"
        case 46:
            return "(46)RESET→STARTで再開してください。"
        case 47:
            return "(47)RESET→STARTで再開してください。"
        case 48:
            return "(48)リールラベル吸着解除失敗"
        case 49:
            return "(49)RESET→STARTで再開してください。"
        case 50:
            return "(50)RESET→STARTで再開してください。"
        case 51:
            return "(51)RESET→STARTで再開してください。"
        case 52:
            return "(52)RESET→STARTで再開してください。"
        case 53:
            return "(53)詰まったリールを取り除き、RESET→STARTで再開してください。"
        case 54:
            return "(54)RESET→STARTで再開してください。"
        case 55:
            return "(55)詰まったリールを取り除き、RESET→STARTで再開してください。"
        case 56:
            return "(56)詰まったリールを取り除き、RESET→STARTで再開してください。"
        case 57:
            return "(57)失敗したリールのラベルを確認し、問題なくラベルが貼られていれば元の場所に戻し、設定2より現在本数を1増やしRESET→STARTで再開してください。"
        case 58:
            return "(58)失敗したリールのラベルを確認し、問題なくラベルが貼られていれば元の場所に戻し、設定2より現在本数を1増やしRESET→STARTで再開してください。"
        case 59:
            return "(59)失敗したリールのラベルを確認し、問題なくラベルが貼られていれば元の場所に戻し、設定2より現在本数を1増やしRESET→STARTで再開してください。"
        case 60:
            return "(60)RESET→STARTで再開してください。"
        case 61:
            return "(61)パネル上のリール本数とストックされた本数が合っているか確認し、問題がなければRESET→STARTで再開してください。"
        case 62:
            return "(62)RESET→STARTで再開してください。"
        case 63:
            return "(63)RESET→STARTで再開してください。"
        case 64:
            return "(64)RESET→STARTで再開してください。"
        case 65:
            return "(65)RESET→STARTで再開してください。"
        case 66:
            return "(66)箱ストック内部で箱の詰まり・傾きを確認し、問題がなければRESET→STARTで再開してください。"
        case 67:
            return "(67)箱が補充されているか確認しRESET→STARTで再開してください。"
        case 68:
            return "(68)詰まっている箱をベルトから取り除き、RESET→STARTで再開してください。"
        case 69:
            return "(69)RESET→STARTで再開してください"
        case 70:
            return "(70)詰まっている箱をベルトから取り除き、RESET→STARTで再開してください。"
        case 71:
            return "(71)RESET→STARTで再開してください。"
        case 72:
            return "(72)詰まっている箱をベルトから取り除き、RESET→STARTで再開してください。"
        case 73:
            return "(73)RESET→STARTで再開してください。"
        case 74:
            return "(74))詰まっている箱をベルトから取り除き、RESET→STARTで再開してください。"
        case 75:
            return "(75)RESET→STARTで再開してください"
        case 76:
            return "(76)供給された箱の向きを合わせ、RESET→STARTで再開してください。"
        case 77:
            return "(77)平箱の位置を修正しRESET→STARTで再開してください。"
        case 78:
            return "(78)平箱の位置を修正しRESET→STARTで再開してください。"
        case 79:
            return "(79)平箱の位置を修正しRESET→STARTで再開してください。"
        case 80:
            return "(80)平箱の位置を修正しRESET→STARTで再開してください。"
        case 81:
            return "(81)フラップモータの状態を確認し、問題がなければRESET→STARTで再開してください。"
        case 82:
            return "(82)RESET→STARTで再開してください。"
        case 83:
            return "(83)平箱の位置を修正しRESET→STARTで再開してください。"
        case 84:
            return "(84)RESET→STARTで再開してください。"
        case 85:
            return "(85)RESET→STARTで再開してください。"
        case 86:
            return "(86)箱の位置を修正しRESET→STARTで再開してください。"
        case 87:
            return "(87)箱の位置を修正しRESET→STARTで再開してください。"
        case 88:
            return "(88)箱の位置を修正しRESET→STARTで再開してください。"
        case 89:
            return "(89)RESET→STARTで再開してください。"
        case 90:
            return "(90)RESET→STARTで再開してください。"
        case 91:
            return "(91)RESET→STARTで再開してください。"
        case 92:
            return "(92)RESET→STARTで再開してください。"
        case 93:
            return "(93)RESET→STARTで再開してください。"
        case 94:
            return "(94)RESET→STARTで再開してください。"
        case 95:
            return "(95)RESET→STARTで再開してください。"
        case 96:
            return "(96)RESET→STARTで再開してください。"
        case 97:
            return "(97)RESET→STARTで再開してください。"
        case 98:
            return "(98)フラップモータの状態を確認し、問題がなければRESET→STARTで再開してください。"
        case 99:
            return "(99)プリンタの状態を確認し、RESET→STARTで再開してください。"
        case 100:
            return "(100)箱ラベルがプリンタ台にあることを確認し、RESET→STARTで再開してください。"
        case 101:
            return "(101)RESET→STARTで再開してください。"
        case 102:
            return "(102)箱ラベルがプリンタ台にあることを確認し、RESET→STARTで再開してください。"
        case 103:
            return "(103)RESET→STARTで再開してください。"
        case 104:
            return "(104)RESET→STARTで再開してください。"
        case 105:
            return "(105)RESET→STARTで再開してください。"
        case 106:
            return "(106)RESET→STARTで再開してください。"
        case 107:
            return "(107)RESET→STARTで再開してください。"
        case 108:
            return "(108)RESET→STARTで再開してください。"
        case 109:
            return "(109)RESET→STARTで再開してください。"
        case 110:
            return "(110)RESET→STARTで再開してください。"
        case 111:
            return "(111)RESET→STARTで再開してください。"
        case 112:
            return "(112)RESET→STARTで再開してください。"
        case 113:
            return "(113)装置の電源を再起動してください。"
        case 114:
            return "(114)RESET→STARTで再開してください。"
        case 115:
            return "(115)詰まったリールを除去し、装置の電源を再起動してください。"
        case 116:
            return "(116)装置の電源を再起動してください。"
        case 117:
            return "(117)RESET→STARTで再開してください。"
        case 118:
            return "(118)RESET→STARTで再開してください。"
        case 119:
            return "(119)詰まったリールを除去し、アームを上に戻し装置の電源を再起動してください。"
        case 120:
            return "(120)詰まったリールを除去し、アームを上に戻し装置の電源を再起動してください。"
        case 121:
            return "(121)RESET→STARTで再開してください。"
        case 122:
            return "(122)RESET→STARTで再開してください。"
        case 123:
            return "(123)RESET→STARTで再開してください。"
        case 124:
            return "(124)失敗した箱を取り除きRESET→STARTで再開してください。"
        case 125:
            return "(125)アーム周りに問題がなければRESET→STARTで再開してください。"
        case 126:
            return "(126)RESET→STARTで再開してください。"
        case 127:
            return "(127)RESET→STARTで再開してください。"
        case 128:
            return "(128)RESET→STARTで再開してください。"
        case 129:
            return "(129)RESET→STARTで再開してください。"
        case 130:
            return "(130)RESET→STARTで再開してください。"
        case 131:
            return "(131)RESET→STARTで再開してください。"
        case 132:
            return "(132)RESET→STARTで再開してください。"
        case 133:
            return "(133)RESET→STARTで再開してください。"
        case 134:
            return "(134)RESET→STARTで再開してください。"
        case 135:
            return "(135)アーム周りに問題がなければRESET→STARTで再開してください。"
        case 136:
            return "(136)RESET→STARTで再開してください。"
        case 137:
            return "(137)排出された箱を取り除きRESET→STARTで再開してください。"
        case 138:
            return "(138)箱を取り除きRESET→STARTで再開してください。"
        case 139:
            return "(139)RESET→STARTで再開してください。"
        case 140:
            return "(140)RESET→STARTで再開してください。"
        case 141:
            return "(141)アーム周りに問題がなければRESET→STARTで再開してください。"
        case 142:
            return "(142)RESET→STARTで再開してください。"
        case 143:
            return "(143)STARTで再開してください"
        case 144:
            return "(144)nani"
        case 145:
            return "(145)nani"
        case 146:
            return "(146)nani"
        case 147:
            return "(147)箱詰めされた製品を取り、RESET→STARTで再開してください。"
        case 148:
            return "(148)倒れたリールを起こしRESET→STARTで再開してください。"
        case 149:
            return "(149)落下したリールを取りRESET→STARTで再開してください。"
        case 150:
            return "(150)ストックされたリールの本数とパネルに表示されたリール本数を確認し、合っていればRESETを3秒以上長押しして解除→STARTで再開してください。"
        case 151:
            return "(151)問題がなければRESET→STARTで再開してください。"
        case 152:
            return "(152)問題がなければRESET→STARTで再開してください。"
        case 153:
            return "(153)RESET→STARTで再開してください。"
        case 154:
            return "(154)RESET→STARTで再開してください。"
        case 155:
            return "(155))排出されたリール・吸着中のリールラベルを取り除き、5本ストック部に赤リールを入れRESET→STARTで再開してください。"

        case 200:
            return "(200)積み重なったリールの一枚を抜き取り、装置の電源を再起動してください。"
        case 201:
            return "(201)エアーが入っているか確認し、装置の電源を再起動してください。"
        case 202:
            return "(202)装置の電源を再起動してください。"
        case 203:
            return "(203)装置の電源を再起動してください。"
        case 204:
            return "(204)装置の電源を再起動してください。"
        case 205:
            return "(205)装置の電源を再起動してください。"
        case 206:
            return "(206)装置の電源を再起動してください。"
        case 207:
            return "(207)装置の電源を再起動してください。"
        case 208:
            return "(208)装置の電源を再起動してください。"
        case 209:
            return "(209)装置の電源を再起動してください。"
        case 210:
            return "(210)装置の電源を再起動してください。"
        case 211:
            return "(211)装置の電源を再起動してください。"
        case 212:
            return "(212)装置の電源を再起動してください。"
        case 213:
            return "(213)装置の電源を再起動してください。"
        case 214:
            return "(214)装置の電源を再起動してください。"
        case 215:
            return "(215)装置の電源を再起動してください。"
        case 216:
            return "(216)装置の電源を再起動してください。"
        case 217:
            return "(217)装置の電源を再起動してください。"
        case 218:
            return "(218)装置の電源を再起動してください。"
        case 219:
            return "(219)装置の電源を再起動してください。"
        case 220:
            return "(220)装置の電源を再起動してください。"
        case 221:
            return "(221)装置の電源を再起動してください。"
        case 222:
            return "(222)装置の電源を再起動してください。"
        case 223:
            return "(223)装置の電源を再起動してください。"
        case 224:
            return "(224)装置の電源を再起動してください。"
        case 225:
            return "(225)装置の電源を再起動してください。"
        case 226:
            return "(226)装置の電源を再起動してください。"
        case 227:
            return "(227)装置の電源を再起動してください。"
        case 228:
            return "(228)装置の電源を再起動してください。"
        case 229:
            return "(229)装置の電源を再起動してください。"
        case 230:
            return "(230)装置の電源を再起動してください。"
        case 231:
            return "(231)装置の電源を再起動してください。"
        case 232:
            return "(232)装置の電源を再起動してください。"
        case 233:
            return "(233)装置の電源を再起動してください。"
        case 234:
            return "(234)装置の電源を再起動してください。"
        case 235:
            return "(235)装置の電源を再起動してください。"
        case 236:
            return "(236)装置の電源を再起動してください。"
        case 237:
            return "(237)装置の電源を再起動してください。"
        case 238:
            return "(238)装置の電源を再起動してください。"
        case 239:
            return "(239)装置の電源を再起動してください。"
        case 240:
            return "(240)装置の電源を再起動してください。"
        case 241:
            return "(241)装置の電源を再起動してください。"
        case 242:
            return "(242)装置の電源を再起動してください。"
        case 243:
            return "(243)装置の電源を再起動してください。"
        case 250:
            return "(250)箱供給部に残った箱を取り除き装置の電源を再起動してください。"
        case 251:
            return "(251)装置の電源を再起動し原点復帰を完了させてください。"

        case 301:
            return "ロット交換時、RESETを4.5秒以上長押しして解除してください。"
        case 302:
            return "(302)リールとラベルの総数を確認してください。"
        case 303:
            return "(303)ラベルデータを入力してください。"
        case 304:
            return "(304)リールとラベルの総数を確認してください"
        case 305:
            return "(305)自動モードに切り替え、RESET→STARTで再開してください。"
        case 306:
            return "(306)倒れたリールを起こしRESET→STARTで再開してください。"
        case 307:
            return "(307)リールラベルが残っていないか確認し、なければRESET→STARTで再開してください。"
        case 308:
            return "(308)非常停止スイッチを解除し、RESET→STARTで再開してください。"
        case 309:
            return "(309)落下した箱を取り除きRESET→STARTで再開してください。"
        case 310:
            return "(310)パネルの指示に従い復帰動作を行ってください。"
        case 311:
            return "(311)箱が正しく入れ直されているか確認し再度パネルの指示に従ってください。"
        case 312:
            return "(312)RESET→STARTで再開してください。"
        case 313:
            return "(313)RESET→STARTで再開してください。"
        case _:
            return ""

def Disp_draw(self):
    loop = 0
    if self.tab_select == 0 and self.error_code.get() == 0:
        self.set_image_main("labelimage\\zenkikaigood.PNG")
        self.set_image_sita("taiyo.PNG")
        # self.canvas_sita.delete("all")

    if self.tab_select == 0 and self.error_code.get() == 1:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\1.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    ###########
    if self.tab_select == 0 and self.error_code.get() == 2:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\2_8.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    ##################################################
    if self.tab_select == 0 and self.error_code.get() == 3:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\3_6.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 4:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\4.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 5:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\5.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 6:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\3_6.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 7:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\7.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 8:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\2_8.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 9:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\9_19.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 10:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\10_18.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 11:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\11_13_15.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 12:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\12_14_145.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 13:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\11_13_15.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 14:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\12_14_145.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 15:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\11_13_15.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 16:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\16_22_23_25_29_32_38.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 17:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\17_34.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 18:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\10_18.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 19:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\9_19.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 20:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\20_33.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 21:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\21.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 22:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\16_22_23_25_29_32_38.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 23:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\16_22_23_25_29_32_38.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 24:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\24_30.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 25:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\16_22_23_25_29_32_38.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 26:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\26_27_28.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 27:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\26_27_28.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 28:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\26_27_28.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 29:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\16_22_23_25_29_32_38.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 30:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\24_30.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 31:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\31_37.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 32:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\16_22_23_25_29_32_38.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 33:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\20_33.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 34:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\17_34.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 35:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\35_36.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 36:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\35_36.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 37:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\31_37.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 38:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\16_22_23_25_29_32_38.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 39:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\zen.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\print1.PNG")
            self.set_image_sita("labelimage\\39_40.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 40:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\zen.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\print1.PNG")
            self.set_image_sita("labelimage\\39_40.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 41:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\41_43_47_50.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 42:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\42_48.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 43:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\41_43_47_50.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 44:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\44_52.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 45:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\45_51.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 46:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\46_49.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 47:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\41_43_47_50.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 48:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\42_48.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 49:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\46_49.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 50:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\41_43_47_50.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 51:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\45_51.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 52:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\44_52.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 53:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\53.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 54:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\54_57_58.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 55:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\55_62.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 56:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\56_63.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 57:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\54_57_58.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 58:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\54_57_58.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 59:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\59.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 60:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\60_64.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 61:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\61_65.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 62:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\55_62.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 63:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\56_63.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 64:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\60_64.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 65:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\61_65.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 66:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\zen.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\66_67.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 67:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\zen.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\66_67.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 68:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\68_69_75.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 69:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\68_69_75.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 70:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\70.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 71:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\71.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 72:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\72.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 73:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\73.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 74:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\74_76_83.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 75:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\68_69_75.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 76:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\74_76_83.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 77:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\77_80_84.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 78:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\78_79_85.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 79:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\78_79_85.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 80:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\77_80_84.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 81:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\81_86_87_88_98.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 82:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\82_97.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 83:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\74_76_83.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 84:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\77_80_84.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 85:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\78_79_85.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 86:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\81_86_87_88_98.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 87:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\81_86_87_88_98.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 88:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\81_86_87_88_98.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 89:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\89.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 90:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\90.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 91:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\91_92_93_94_95_96.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 92:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\91_92_93_94_95_96.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 93:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\91_92_93_94_95_96.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 94:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\91_92_93_94_95_96.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 95:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\91_92_93_94_95_96.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 96:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\91_92_93_94_95_96.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 97:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\82_97.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 98:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\81_86_87_88_98.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 99:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\zen.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\99_100.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 100:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\zen.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\99_100.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 101:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\101_102_103_107_108_109.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 102:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\101_102_103_107_108_109.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 103:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\101_102_103_107_108_109.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 104:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\104_112.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 105:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\105_110.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 106:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\106_111.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 107:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\101_102_103_107_108_109.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 108:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\101_102_103_107_108_109.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 109:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\101_102_103_107_108_109.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 110:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\105_110.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 111:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\106_111.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 112:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\104_112.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 113:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\113_115_116.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 114:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\114_117.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 115:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\113_115_116.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 116:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\113_115_116.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 117:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\114_117.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 118:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\118_121.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 119:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\119_120.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 120:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\119_120.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 121:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\118_121.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 122:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\122_126.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 123:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\123_132.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 124:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\124_129.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 125:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\125.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 126:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\122_126.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 127:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\zen.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\127_128_130_131_133_134.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 128:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\zen.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\127_128_130_131_133_134.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 129:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\124_129.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 130:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\zen.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\127_128_130_131_133_134.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 131:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\zen.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\127_128_130_131_133_134.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 132:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\123_132.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 133:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\zen.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\127_128_130_131_133_134.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 134:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\zen.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\127_128_130_131_133_134.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 135:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\135_136_137_143.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 136:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\135_136_137_143.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 137:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\135_136_137_143.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 138:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\138_139_140.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 139:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\138_139_140.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 140:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\138_139_140.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 141:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\141_142.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 142:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\141_142.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 143:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.PNG")
            self.set_image_sita("labelimage\\135_136_137_143.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 144:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.png")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\144_145.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 145:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.png")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\12_14_145.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 146:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.png")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\146.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 147:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\full.PNG")
            self.set_image_sita("labelimage\\zen.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\seihin.PNG")
            self.set_image_sita("labelimage\\147.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 148:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\148.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 149:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\149.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 150:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\150.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\150.PNG")
            self.set_image_sita("labelimage\\148.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 151:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\151_152_153_154.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 155:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\12_14_145.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    ###############################################
    #################GENTEN########################
    if self.tab_select == 0 and self.error_code.get() == 200:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.PNG")
            self.set_image_sita("labelimage\\200.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 201:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\201.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zen.PNG")
            self.set_image_sita("labelimage\\201.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 202:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\202.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 203:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\203.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 204:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\204.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    if self.tab_select == 0 and self.error_code.get() == 205:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\205.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 206:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\206.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 207:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\207.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 208:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\208.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 209:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\209.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 210:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\210.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 211:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\211.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 212:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\212.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 213:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\213.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 214:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\214.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 215:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\215.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 216:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\216.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 217:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\217.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 218:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\218.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 219:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\219.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 220:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\220.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 221:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\221.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 222:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.png")
            self.set_image_sita("labelimage\\222.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 223:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.png")
            self.set_image_sita("labelimage\\223.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 224:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.png")
            self.set_image_sita("labelimage\\224.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 225:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.png")
            self.set_image_sita("labelimage\\225.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 226:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.png")
            self.set_image_sita("labelimage\\226.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 227:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.png")
            self.set_image_sita("labelimage\\227.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 228:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.png")
            self.set_image_sita("labelimage\\228.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 229:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.png")
            self.set_image_sita("labelimage\\229.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 230:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.png")
            self.set_image_sita("labelimage\\230.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 231:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\haako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.png")
            self.set_image_sita("labelimage\\231.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 232:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\232.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 233:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\onmote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\233.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 234:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\234.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 235:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.png")
            self.set_image_sita("labelimage\\235_236.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 236:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.png")
            self.set_image_sita("labelimage\\235_236.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 237:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\237_238.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 238:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\237_238.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 239:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\239.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 240:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\240.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 241:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\241.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 242:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\242.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 243:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.png")
            self.set_image_sita("labelimage\\243.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 250:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\hako1.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\hako.png")
            self.set_image_sita("labelimage\\250.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 300:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zen.PNG")
            self.set_image_sita("labelimage\\genten.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\genten.png")
            self.set_image_sita("labelimage\\zen.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 301:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zen.PNG")
            self.set_image_sita("labelimage\\rotto.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\rotto.png")
            self.set_image_sita("labelimage\\zen.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 302:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zen.PNG")
            self.set_image_sita("labelimage\\302_304.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\302_304.png")
            self.set_image_sita("labelimage\\zen.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 303:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zen.PNG")
            self.set_image_sita("labelimage\\303.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\303.png")
            self.set_image_sita("labelimage\\zen.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 304:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zen.PNG")
            self.set_image_sita("labelimage\\302_304.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\302_304.png")
            self.set_image_sita("labelimage\\zen.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 306:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\306.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 307:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\ura.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\307.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 312:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\312.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1

    if self.tab_select == 0 and self.error_code.get() == 313:

        if self.loop_display == 1:
            self.set_image_main("labelimage\\zenkikainotgood.PNG")
            self.set_image_sita("labelimage\\omote.PNG")
        if self.loop_display == 2:
            self.set_image_main("labelimage\\zenkikaing.png")
            self.set_image_sita("labelimage\\313.PNG")

        self.loop_display = self.loop_display + 1
        if self.loop_display > 2:
            self.loop_display = 1
    ###############################################
    ###############################################

    if self.tab_select == 2:
        self.set_image_hoshu1("cautao.PNG")
    if self.tab_select == 3:
        self.set_image_hoshu2("hako.PNG")

    self.draw_id = self.after(500, self.Disp_draw)