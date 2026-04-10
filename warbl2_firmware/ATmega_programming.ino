
// Reprogram the ATmega32u4 if necessary.
// Modified from the Adafruit_AVRProg library.
// LED will flash teal 3 times to indicate success.
// To use, paste in the ATmega firmware hex file below, make sure HEX_SIZE is set large enough (otherwise it won't compile), and set the new ATmega firmware version in Defines.h so we know the ATmega will need to be programmed.

#define HEX_SIZE 20000  // This needs to be set larger than the size of the hex file in bytes. If it is larger than necessary the buffer will take up unneeded space in flash.

#define FUSE_PROT 0               // Memory protection
#define FUSE_LOW 1                // Low fuse
#define FUSE_HIGH 2               // High fuse
#define FUSE_EXT 3                // Extended fuse
#define FUSE_CLOCKSPEED 10000     // Fuses need to be programmed slowly.
#define FLASH_CLOCKSPEED 1000000  // Once fused you can flash fast!
#define AVRPROG_RESET 26          // This pin is left in default state (highZ) when not programming (it has a pullup resistor).
#define debug(string)             // Serial.println(string);


typedef struct image {             // Struct for holding program fuses & code
    uint16_t image_chipsig;        // Low two bytes of signature
    byte image_progfuses[10];      // Fuses to set during programming (e.g unlock)
    byte image_normfuses[10];      // Fuses to set after programming (e.g lock)
    byte fusemask[10];             // Not all bits are used in the fuses, mask the ones we do use.
    uint16_t chipsize;             // Total size for flash programming, in bytes.
    byte image_pagesize;           // Page size for flash programming, in bytes.
    byte image_hexcode[HEX_SIZE];  // Max buffer for intel hex format image (text)
} image_t;

static byte pageBuffer[8 * 1024];  // Megabuff
extern const image_t *images[];

// The firmware hex file to flash. Paste it in between the start and end markers {R"( and )"}.
const image_t PROGMEM image_32u4_boot = { 0x9587, { 0x3F, 0xFF, 0xD8, 0x0B }, { 0x2F, 0xFF, 0xD9, 0x0B }, { 0x3F, 0xFF, 0xFF, 0x0F }, 32768, 128,  // Signature, programming fuses, final fuses, fuse verify mask, flash size, page size
                                          { R"(
:100000000C94DD000C94EF060C94C8060C94A10629
:100010000C947A060C9405010C9405010C9453067B
:100020000C9405010C9405010C945E030C94C70319
:100030000C9405010C9405010C9405010C94050128
:100040000C9405010C9405010C9405010C94050118
:100050000C9405010C9405010C9405010C940906FF
:100060000C9417070C9405010C9405010C940501E0
:100070000C9405010C9416070C9405010C940501D1
:100080000C9405010C9405010C9405010C940501D8
:100090000C9405010C9405010C9405010C940501C8
:1000A0000C9405010C9405010C9405010706050448
:1000B0000100080A0B0C0D090000000025002800B3
:1000C0002B002E00310000000000240027002A0031
:1000D0002D003000000000002300260029002C0025
:1000E0002F000404040404030405020202020403B2
:1000F00002020202060606060606040402020204C2
:100100000404080201104080401020408040800814
:100110000204018040201002011080102040402085
:100120000000000200090F0000030401000C0000A1
:1001300000000000000000000000000000000008B7
:100140000B0002020200000904000001020200008C
:1001500005240010010524010101042402060524E0
:100160000600010705810310004009040100020A8E
:10017000000000070502024000000705830240005E
:10018000000403090412010002EF020140D80487B1
:10019000EE0001010203014D6F77727920537472F2
:1001A000696E67656420496E737472756D656E74EF
:1001B0007300574152424C003A0711241FBECFEF43
:1001C000DAE0DEBFCDBF11E0A0E0B1E0E0E9F6E1AA
:1001D00002C005900D92AC34B107D9F721E0ACE430
:1001E000B1E001C01D92AB3EB207E1F710E0CDEDEA
:1001F000D0E004C02197FE010E94400BCC3DD10706
:10020000C9F70E944F070C94460B0C940000909184
:10021000EA01911103C08FEF9FEF08954FB7F89453
:1002200092E09093E9009091F200292F30E01216AD
:10023000130614F421E030E0232B29F084E68093A8
:10024000E9018091F100992339F02091F200211108
:1002500003C02BE62093E8004FBF9923E1F290E022
:1002600008952FB7F89483E08093E9009091E80017
:10027000892F807295FF04C09091F20080E4891B61
:100280002FBF08958091E10181110DC082E080931C
:10029000DD0184E08093DE011092E0011092DF0125
:1002A00081E08093E1018DED91E00895CF93DF939C
:1002B0001F92CDB7DEB76983DC01ED91FC9102801E
:1002C000F381E02D41E050E0BE016F5F7F4F099563
:1002D0000F90DF91CF91089583E08093E9008091A2
:1002E000F200882319F08AE38093E80008950E94C1
:1002F000310190E00895FC018485958597FD05C046
:100300002FEF3FEF3587248708950C940701CF9393
:10031000DF93EC018C859D8597FF04C00E94070147
:100320009D878C878C859D85DF91CF910895FC01F9
:100330008485958597FD0BC09FB7F89482E08093E4
:10034000E9008091F2009FBF90E0019608959FB769
:10035000F89482E08093E9008091F2009FBF90E0E2
:100360000895409187015091880120918501309135
:10037000860142175307B4F49091E8009570E1F3B9
:100380009091E80092FD19C08093F100809187015F
:100390009091880101968F739927892B19F48EEF1C
:1003A0008093E80080918701909188010196909355
:1003B00088018093870181E0089580E00895EF929D
:1003C000FF920F931F93CF93DF93F82E192FE62EF2
:1003D000042F81E0860F880F0E94B10183E00E9404
:1003E000B101CF2DD12FEC0EFD2EF11CCE15DF0566
:1003F000B9F007FF13C0FE0184910E94B101182FCC
:1004000080E00E94B101812321968111EFCFDF911D
:10041000CF911F910F91FF90EF9008958881EDCFBC
:1004200081E0F5CFDF92EF92FF920F931F93CF936E
:10043000DF93D82E8A01EB017B01E40EF51ECE1569
:10044000DF0559F0D7FE12C0FE0184910E94B10170
:1004500021968111F4CF0FEF1FEFC801DF91CF91EB
:100460001F910F91FF90EF90DF9008958881EECF5C
:100470000F931F93CF93DF931F92CDB7DEB782E028
:10048000898342E450E06FE371E080E80E94120249
:100490000E944201DC0112960D911C91011511057B
:1004A00089F0D801ED91FC910280F381E02DBE012D
:1004B0006F5F7F4FC801099597FD04C0F801008563
:1004C0001185ECCF89810F90DF91CF911F910F9112
:1004D00008953FB7F8948091810190918201A09195
:1004E0008301B091840126B5A89B05C02F3F19F068
:1004F0000196A11DB11D3FBFBA2FA92F982F8827A4
:10050000BC01CD01620F711D811D911D43E0660F7D
:10051000771F881F991F4A95D1F70895CF92DF92D0
:10052000EF92FF920E9469026B017C010E946902B6
:100530006C197D098E099F09683E734081059105FC
:10054000A8F3FF90EF90DF90CF9008958F929F9245
:10055000AF92BF92CF92DF92EF92FF920F931F93D1
:10056000CF93DF936C017B018A0180913101882355
:1005700009F457C08091EA01882309F452C08091A0
:100580008C0180FF05C08091E00082608093E000D4
:10059000E801B12C8AEFA82E93E0892E2AE3922E4F
:1005A000209711F4BB20B9F10E943101811106C0DE
:1005B000AA94AA20B1F10E948E02F2CF8C171D06D8
:1005C00011F00CF08C2F9FB7F8948092E9002091E5
:1005D000E80025FD02C09FBFE3CF282F30E0C21BFB
:1005E000D30BF701815020F041914093F100FACFF5
:1005F000E20EF31EBB2021F09092E800B12CEBCF6D
:100600008091E80085FDE7CF9092E800BB24B39489
:10061000209709F3F3CF84E680938B01101611061F
:100620003CF081E090E0F6019383828310E000E0EB
:10063000C801DF91CF911F910F91FF90EF90DF9054
:10064000CF90BF90AF909F908F90089583B78E7F8B
:1006500083BF8FEF8EBD08950895823108F08251D7
:10066000E82FF0E0E455FF4FE49190917B008E2F4E
:10067000880F880F80729F7D892B80937B00E770A5
:10068000E064E0937C00F89483B7817F826083BF4D
:1006900083B7816083BF80917A00886480937A00F9
:1006A0007894889583B78E7F83BF80917A0086FD8A
:1006B000FCCF809178009091790008951F920F925D
:1006C0000FB60F9211248F939F938091E100909128
:1006D000E100937F9093E10083FF0FC01092E90047
:1006E00091E09093EB001092EC0092E39093ED0078
:1006F0001092EA0198E09093F00082FF1CC093E012
:100700009093E9009091F200992319F09AE3909365
:10071000E80090918B01992329F090918B019150E1
:1007200090938B019091E901992329F09091E9012F
:1007300091509093E90184FF18C08091E2008E7E71
:1007400081608093E2008091E1008F7E8093E100E0
:1007500080918C018E7E806180938C019F918F911E
:100760000F900FBE0F901F90189580FFF7CF8091CC
:10077000E2008E7E80618093E2008091E1008E7EB7
:100780008093E10080918C018E7E8160E5CF1F9285
:100790000F920FB60F921124CF92DF92EF92FF9239
:1007A0000F931F932F933F934F935F936F937F9379
:1007B0008F939F93AF93BF93EF93FF93CF93DF9369
:1007C000CDB7DEB76C97DEBFCDBF1092E900809148
:1007D000E80083FF2FC089E0FE017596EE2EDF2E24
:1007E000815029F09091F10090833196F9CF84E601
:1007F0008093E90182EF8093E8008D8987FF39C0FB
:100800009091E80090FFFCCF982F907609F033C1CB
:100810009E894F89588D2F89F88C911131C080386D
:1008200061F580918A018093F1001092F1008EEFC2
:100830008093E8006C960FB6F894DEBF0FBECDBF74
:10084000DF91CF91FF91EF91BF91AF919F918F91E8
:100850007F916F915F914F913F912F911F910F91D8
:10086000FF90EF90DF90CF900F900FBE0F901F90F2
:1008700018959EEF9093E800C7CF1092F100D5CF66
:10088000913059F48111D3CF4130510581F68091D7
:100890008A018D7F80938A01CACF933049F48111F8
:1008A000C6CF4130510519F680918A018260F2CF9E
:1008B000953041F48091E80080FFFCCF20682093C0
:1008C000E300B5CF963009F0A8C00B8D1C8D22E057
:1008D0001092E9001092880110928701F2122EC046
:1008E00010928601109285010E9438021F8299E0C1
:1008F0009983FA8291E09E8390EA98879AE099879B
:100900002091870130918801275F3F4F3C832B83E3
:100910008D831092E9001092880110928701109344
:1009200086010093850149E050E0BE016F5F7F4F73
:1009300080E00E9412020E94380279CF1093860153
:10094000009385010E944201DC0112960D911C91D9
:100950000115110509F457C1D801ED91FC910480EE
:10096000F581E02D6E2D7D2DC8010995009709F0C8
:1009700045C1F80100851185EBCFF3E0FF120EC0F1
:100980008F89882309F440C0823061F440E865E033
:1009900082EB91E00E94DF01811149CF81E28093D7
:1009A000EB0048CF813029F440E86AE187E991E023
:1009B000F1CF833099F70E944201DC011296ED904D
:1009C000FC908E010F5F1F4F6801E114F10479F074
:1009D000D701ED91FC910680F781E02DB801C701A8
:1009E0000995080F111DF701E084F184EECFD801BD
:1009F0001C92F60101900020E9F73197BF016C19B4
:100A00007D0940E0C601C6CF61E871E0FB01449179
:100A100050E080E80E9412020ACF973009F4BECF5E
:100A2000983021F481E08093F10001CF993009F0F2
:100A3000FECE837009F0B2CFE3E3F1E081E031E074
:100A400096E32191222371F08093E9003093EB002B
:100A5000DF0111972C912093EC009093ED008F5FB4
:100A6000873079F78EE78093EA001092EA008F8949
:100A70008093EA01DCCE8B8D9C8D1092E900109260
:100A80008801109287019093860180938501898D5A
:100A900081119AC08E899D89913A49F4813209F079
:100AA0007DCF47E050E06AE271E080E0B3CF913261
:100AB00009F074CF833269F48F89988DB0E0A0E09B
:100AC0008093260190932701A0932801B0932901D8
:100AD000AECE8032A9F48091E80082FFFCCF88E09E
:100AE000EAE2F1E0815029F09091F1009083319693
:100AF000F9CF84E68093E9018BEF8093E80097CEED
:100B0000823209F094CE8F8980933101EEEFFFE7B6
:100B1000859194918B3F9C4D51F1E0E0F8E08091FC
:100B20002A0190912B01A0912C01B0912D01803BC5
:100B30009440A105B105F1F48091310180FD1AC006
:100B4000EE3F8AE0F80789F587E797E7918380838E
:100B5000809160008093890188E19BE00FB6F89452
:100B6000A895809360000FBE9093600060CEEEEF7A
:100B7000FAE0D5CF808191818737974709F057CE2A
:100B8000A8958091600088618093600080918901C0
:100B900080936000EE3F2AE0F20789F08091FE0A20
:100BA0009091FF0A9183808342CE80819181873723
:100BB000980751F29093FF0A8093FE0AC5CF1092D6
:100BC000FF0A1092FE0A33CE0E944201DC01129607
:100BD0000D911C910115110509F4E0CED801ED919C
:100BE000FC910190F081E02D6E2D7D2DC8010995BD
:100BF00081111DCEF80100851185ECCF181619065C
:100C00000CF415CECBCEF1E0FF12B7CE65E871E063
:100C1000FDCE1F920F920FB60F9211242F933F9388
:100C20008F939F93AF93BF9380917D0190917E01AD
:100C3000A0917F01B091800130917C0126E0230FCB
:100C40002D3758F50296A11DB11D20937C0180938C
:100C50007D0190937E01A0937F01B09380018091EC
:100C6000810190918201A0918301B091840101964C
:100C7000A11DB11D8093810190938201A0938301F6
:100C8000B0938401BF91AF919F918F913F912F912C
:100C90000F900FBE0F901F90189529E8230F039611
:100CA000A11DB11DD2CF1F920F920FB60F9211242A
:100CB0002F933F934F935F936F937F938F939F9364
:100CC000AF93BF93EF93FF93E0910801F091090177
:100CD0000995FF91EF91BF91AF919F918F917F9176
:100CE0006F915F914F913F912F910F900FBE0F9099
:100CF0001F9018951F920F920FB60F9211242F93E9
:100D00003F934F935F936F937F938F939F93AF9393
:100D1000BF93EF93FF93E0910601F09107010995CE
:100D2000FF91EF91BF91AF919F918F917F916F91C3
:100D30005F914F913F912F910F900FBE0F901F9099
:100D400018951F920F920FB60F9211242F933F9375
:100D50004F935F936F937F938F939F93AF93BF93C3
:100D6000EF93FF93E0910401F09105010995FF9144
:100D7000EF91BF91AF919F918F917F916F915F9113
:100D80004F913F912F910F900FBE0F901F9018958C
:100D90001F920F920FB60F9211242F933F934F93F0
:100DA0005F936F937F938F939F93AF93BF93EF93D3
:100DB000FF93E0910201F09103010995FF91EF91FA
:100DC000BF91AF919F918F917F916F915F914F9163
:100DD0003F912F910F900FBE0F901F9018951F926B
:100DE0000F920FB60F9211242F933F934F935F935F
:100DF0006F937F938F939F93AF93BF93EF93FF93E3
:100E0000E0910001F09101010995FF91EF91BF91EF
:100E1000AF919F918F917F916F915F914F913F9192
:100E20002F910F900FBE0F901F90189518951F923D
:100E30000F920FB60F9211248F93EF93FF93EEB59D
:100E4000EC3068F4F0E0E45BFE4F80818EBDFF91F2
:100E5000EF918F910F900FBE0F901F901895E43176
:100E600019F410922501F3CFE53189F781E08093E1
:100E70002501EDCFEDE8F1E01382128288EE93E0D8
:100E8000A0E0B0E084839583A683B7838DE391E0EF
:100E9000918380838FEF9FEF95878487089578945F
:100EA00084B5826084BD84B5816084BD85B582606F
:100EB00085BD85B5816085BD80916E008160809320
:100EC0006E00109281008091810082608093810089
:100ED0008091810081608093810080918000816099
:100EE0008093800080919100826080939100809136
:100EF0009100816080939100809190008160809347
:100F000090008091C10084608093C1008091C100F5
:100F100082608093C1008091C10081608093C10094
:100F20008091C30081608093C3008091C000826083
:100F30008093C0008091C20081608093C200809144
:100F40007A00846080937A0080917A008260809336
:100F50007A0080917A008E7F80937A0080917A0067
:100F6000806880937A001092EA0110928A011092B0
:100F70008C018091D70081608093D70080EA8093B4
:100F8000D80089B58F7E89BD89B5826089BD09B4D5
:100F900000FEFDCF0E948E028091D8008F7C806180
:100FA0008093D8008091E000807F8093E000809162
:100FB000E1008E7E8093E1008DE08093E2008091DD
:100FC000D80080628093D80089B58D7F89BD8091DB
:100FD000D8008F778093D8008091650080688093D7
:100FE0006500809164008260809364008091640059
:100FF00080628093640080916400886080936400C4
:1010000080916400806480936400809165008860B2
:1010100080936500809165008160809365008FEF0B
:1010200080937E0083EF80937D0090E080E0FC0160
:10103000EE0FFF1FED5EFE4F208131814FB7F89418
:10104000F901FF272291232B20834FBF019689307E
:10105000910569F780917A00887F80937A0084E80F
:1010600080937A0080E480937C008CB580648CBD92
:10107000EFE0F1E02491E0EFF0E08491882399F033
:1010800090E0880F991FFC01EA53FF4FA591B4919E
:10109000FC01E854FF4F859194918FB7F894EC913F
:1010A000E22BEC938FBF8CB580688CBDE4E0F1E05F
:1010B0002491E5EEF0E084918823C1F090E0880F60
:1010C000991FFC01EA53FF4FC591D491FC01E854EC
:1010D000FF4FA591B4919FB7F8943881822F8095E6
:1010E00083238883EC91E22BEC939FBF86E293E00D
:1010F0009093010180930001809169008C7F826050
:1011000080936900E89A91E3892E91E0992E02E19B
:1011100011E023EFE22E20E0F22E37E5C32E31E07E
:10112000D32E40E0A42E40E0B42E83B7817F8C60A4
:1011300083BFF8948091640081608093640083B7DA
:10114000816083BF78948895F4019491F80124918B
:10115000F70184918823C1F39923E9F091509F30DE
:10116000D0F4E92FF0E0E954F74F0C94400B1B0B3F
:101170001F0BC608130B170BCB08CB08CB08220B91
:10118000280B2C0B300B360BCB083A0B90918000C0
:101190009F7790938000E82FF0E0EE0FFF1FEC5256
:1011A000FF4FA591B4918C91282309F4CDCF809164
:1011B00064008E7F8093640080912501882381F0F4
:1011C00080911301909114012FB7F894FC01FF272F
:1011D0008081892B80832FBF80910A010E942D037B
:1011E00080910B010E942D0390936D0180936C01FF
:1011F00080910A010E942D0320916A0130916B01B8
:10120000821B930B90935901809358018091250183
:10121000811104C0109259011092580120911301BC
:10122000309114019FB7F894F901FF272081832F93
:101230008095822380839FBF809115019091160134
:101240002FB7F894FC01FF278081892B80832FBF63
:1012500080910C010E942D0390936F0180936E0189
:1012600080910B010E942D0320916C0130916D0142
:10127000821B930B90935B0180935A01209115017F
:10128000309116019FB7F894F901FF272081832F31
:101290008095822380839FBF8091170190911801D0
:1012A0002FB7F894FC01FF278081892B80832FBF03
:1012B00080910D010E942D03909371018093700124
:1012C00080910C010E942D0320916E0130916F01DD
:1012D000821B930B90935D0180935C012091170119
:1012E000309118019FB7F894F901FF272081832FCF
:1012F0008095822380839FBF8091190190911A016C
:101300002FB7F894FC01FF278081892B80832FBFA2
:1013100080910E010E942D039093730180937201BE
:1013200080910D010E942D03209170013091710177
:10133000821B930B90935F0180935E0120911901B2
:1013400030911A019FB7F894F901FF272081832F6C
:101350008095822380839FBF80911B0190911C0107
:101360002FB7F894FC01FF278081892B80832FBF42
:1013700080910F010E942D03909375018093740159
:1013800080910E010E942D03209172013091730112
:10139000821B930B909361018093600120911B014C
:1013A00030911C019FB7F894F901FF272081832F0A
:1013B0008095822380839FBF80911D0190911E01A3
:1013C0002FB7F894FC01FF278081892B80832FBFE2
:1013D000809110010E942D039093770180937601F4
:1013E00080910F010E942D032091740130917501AD
:1013F000821B930B909363018093620120911D01E6
:1014000030911E019FB7F894F901FF272081832FA7
:101410008095822380839FBF80911F01909120013E
:101420002FB7F894FC01FF278081892B80832FBF81
:10143000809111010E942D0390937901809378018E
:10144000809110010E942D03209176013091770147
:10145000821B930B909365018093640120911F017F
:10146000309120019FB7F894F901FF272081832F45
:101470008095822380839FBF8091210190912201DA
:101480002FB7F894FC01FF278081892B80832FBF21
:10149000809112010E942D0390937B0180937A0129
:1014A000809111010E942D032091780130917901E2
:1014B000821B930B90936701809366012091210119
:1014C000309122019FB7F894F901FF272081832FE3
:1014D0008095822380839FBF809123019091240176
:1014E0002FB7F894FC01FF278081892B80832FBFC1
:1014F00080910A010E942D0390936B0180936A01F1
:10150000809112010E942D0320917A0130917B017C
:10151000821B930B909369018093680120912301B2
:10152000309124019FB7F894F901FF272081832F80
:101530008095822380839FBFA8E5B1E0FD01819162
:10154000919197FF05C0CF010297EC011982188293
:10155000D1E0EA36FD0799F7F894E8E5F1E090E08C
:1015600080E0408132969C01245B3E4FE901488334
:10157000019689309105A9F71092550110925601F4
:101580001092570180E09091550111962C9111977E
:101590001296922B90935501833031F09091550122
:1015A000990F990F909355018F5F843061F7E0E6B2
:1015B000F1E09091560121813296922B9093560141
:1015C000873031F090915601990F990F9093560101
:1015D0008F5F883071F78091690180935701809106
:1015E00057018F7080935701ECE4F1E090E0819116
:1015F0009827CE16DF06D9F7809157018370982778
:10160000892F82958F708927909157018295807FCD
:10161000892B809357017894A114B10409F485CDE6
:101620000E94000082CD909180009F7DB2CD90916C
:101630008000977FAECD94B59F7794BDACCD94B527
:101640009F7DFBCF909190009F7790939000A3CDCA
:10165000909190009F7DF9CF90919000977FF5CF6A
:101660009091C0009F779093C00095CD9091C0005D
:101670009F7DF9CF9091C200977F9093C2008BCD50
:10168000EE0FFF1F0590F491E02D0994F894FFCF21
:101690002C032C032C032C032C03000908070B0636
:1016A0000402012E402880284025802B042B08317D
:1016B000013110254001FFFFFFFF00E100000000A5
:1016C000000000C18081000000000000005601A65B
:0C16D0000277016C0197017B018701008B
:00000001FF
)" } };

const image_t *images[] = {
    &image_32u4_boot,
};




bool programATmega(void) {


    if (!targetPower(true)) {
        return false;
    }

    uint16_t signature = readSignature();
    if (signature == 0 || signature == 0xFFFF) {
        return false;
    }

    const image_t *targetimage = images[0];
    if (targetimage->image_chipsig != signature) {
        return false;
    }


    eraseChip();

    if (!programFuses(targetimage->image_progfuses, 5)) {  // Get fuses ready to program.
        return false;
    }


    targetPower(false);  // Disconnect/reconnect after fusing.
    delay(100);
    if (!targetPower(true)) {
        return false;
    }


    if (!writeImage(targetimage->image_hexcode, pgm_read_byte(&targetimage->image_pagesize), pgm_read_word(&targetimage->chipsize))) {
        return false;
    }


    if (!verifyImage(targetimage->image_hexcode)) {  // Verify flash.
        return false;
    }


    if (!programFuses(targetimage->image_normfuses, 5)) {  // Set fuses to 'final' state.
        return false;
    }


    if (!verifyFuses(targetimage->image_normfuses, targetimage->fusemask)) {
        return false;
    }

    for (byte i = 0; i < 3; i++) {  // Indicate success.
        analogWrite(LEDpins[GREEN_LED], 1023);
        analogWrite(LEDpins[BLUE_LED], 1023);
        delay(200);
        analogWrite(LEDpins[GREEN_LED], 0);
        analogWrite(LEDpins[BLUE_LED], 0);
        delay(200);
    }

    return true;  // Success
}




void endProgramMode(void) {
    SPI.endTransaction();
    digitalWrite(AVRPROG_RESET, LOW);
    pinMode(AVRPROG_RESET, INPUT);
}


bool targetPower(bool poweron) {
    if (poweron) {
        Serial.print(F("Starting Program Mode..."));
        if (startProgramMode(100000)) {
            Serial.println(F(" [OK]"));
            return true;
        } else {
            Serial.println(F(" [FAIL]"));
            return false;
        }
    } else {
        endProgramMode();
        return true;
    }
}


bool startProgramMode(uint32_t clockspeed) {
    pinMode(AVRPROG_RESET, OUTPUT);
    digitalWrite(AVRPROG_RESET, HIGH);
    delay(5);
    SPI.beginTransaction(SPISettings(clockspeed, MSBFIRST, SPI_MODE0));
    debug("...spi_init done");
    digitalWrite(AVRPROG_RESET, LOW);
    debug("...isp_transaction");
    uint16_t reply = isp_transaction(0xAC, 0x53, 0x00, 0x00);
    if (reply == 0x5300) {
        debug("...Done");
        return true;
    }
    Serial.print(reply, HEX);
    return false;
}


uint16_t readSignature(void) {
    startProgramMode(FUSE_CLOCKSPEED);
    uint16_t target_type = 0;
    target_type = isp_transaction(0x30, 0x00, 0x01, 0x00);
    target_type <<= 8;
    target_type |= isp_transaction(0x30, 0x00, 0x02, 0x00);
    endProgramMode();
    return target_type;
}


bool eraseChip(void) {
    startProgramMode(FUSE_CLOCKSPEED);
    if ((isp_transaction(0xAC, 0x80, 0, 0) & 0xFFFF) != 0x8000) {  // chip erase
        error(F("Error on chip erase command"));
    }
    busyWait();
    endProgramMode();
    return true;
}


bool programFuses(const byte *fuses, uint8_t num_fuses) {
    startProgramMode(FUSE_CLOCKSPEED);
    byte f;
    Serial.println(F("\nSetting fuses"));

    f = pgm_read_byte(&fuses[FUSE_PROT]);
    if (f) {
        Serial.print(F("\tSet Lock Fuse to: "));
        Serial.println(f, HEX);
        if ((isp_transaction(0xAC, 0xE0, 0x00, f) & 0xFFFF) != 0xE000) {
            return false;
        }
    }
    busyWait();
    f = pgm_read_byte(&fuses[FUSE_LOW]);
    if (f) {
        Serial.print(F("\tSet Low Fuse to: "));
        Serial.println(f, HEX);
        if ((isp_transaction(0xAC, 0xA0, 0x00, f) & 0xFFFF) != 0xA000) {
            return false;
        }
    }
    busyWait();
    f = pgm_read_byte(&fuses[FUSE_HIGH]);
    if (f) {
        Serial.print(F("\tSet High Fuse to: "));
        Serial.println(f, HEX);
        if ((isp_transaction(0xAC, 0xA8, 0x00, f) & 0xFFFF) != 0xA800) {
            return false;
        }
    }
    busyWait();
    f = pgm_read_byte(&fuses[FUSE_EXT]);
    if (f) {
        Serial.print(F("\tSet Ext Fuse to: "));
        Serial.println(f, HEX);
        if ((isp_transaction(0xAC, 0xA4, 0x00, f) & 0xFFFF) != 0xA400) {
            return false;
        }
    }
    busyWait();
    Serial.println();
    endProgramMode();
    return true;
}


bool verifyFuses(const byte *fuses, const byte *fusemask) {
    startProgramMode(FUSE_CLOCKSPEED);
    byte f;
    Serial.println(F("Verifying fuses..."));
    f = pgm_read_byte(&fuses[FUSE_PROT]);
    if (f) {
        uint8_t readfuse = isp_transaction(0x58, 0x00, 0x00, 0x00);  // lock fuse
        readfuse &= pgm_read_byte(&fusemask[FUSE_PROT]);
        Serial.print(F("\tLock Fuse = 0x"));
        Serial.println(readfuse, HEX);
        if (readfuse != f)
            return false;
    }
    f = pgm_read_byte(&fuses[FUSE_LOW]);
    if (f) {
        uint8_t readfuse = isp_transaction(0x50, 0x00, 0x00, 0x00);  // low fuse
        Serial.print(F("\tLow Fuse = 0x"));
        Serial.println(readfuse, HEX);
        readfuse &= pgm_read_byte(&fusemask[FUSE_LOW]);
        if (readfuse != f)
            return false;
    }
    f = pgm_read_byte(&fuses[FUSE_HIGH]);
    if (f) {
        uint8_t readfuse = isp_transaction(0x58, 0x08, 0x00, 0x00);  // high fuse
        readfuse &= pgm_read_byte(&fusemask[FUSE_HIGH]);
        Serial.print(F("\tHigh Fuse = 0x"));
        Serial.println(readfuse, HEX);
        if (readfuse != f)
            return false;
    }
    f = pgm_read_byte(&fuses[FUSE_EXT]);
    if (f) {
        uint8_t readfuse = isp_transaction(0x50, 0x08, 0x00, 0x00);  // ext fuse
        readfuse &= pgm_read_byte(&fusemask[FUSE_EXT]);
        Serial.print(F("\tExt Fuse = 0x"));
        Serial.println(readfuse, HEX);
        if (readfuse != f)
            return false;
    }
    Serial.println();
    endProgramMode();
    return true;
}


bool writeImage(const byte *hextext, uint32_t pagesize, uint32_t chipsize) {
    uint32_t flash_start = 0;
    uint32_t pageaddr = 0;

    while (pageaddr < chipsize && hextext) {
        const byte *hextextpos =
          readImagePage(hextext, pageaddr, pagesize, pageBuffer);

        bool blankpage = true;
        for (uint16_t i = 0; i < pagesize; i++) {
            if (pageBuffer[i] != 0xFF)
                blankpage = false;
        }
        if (!blankpage) {
            if (!flashPage(pageBuffer, flash_start + pageaddr, pagesize))
                return false;
        }
        hextext = hextextpos;
        pageaddr += pagesize;
    }
    return true;
}


const byte *readImagePage(const byte *hextext,
                          uint16_t pageaddr,
                          uint16_t pagesize,
                          byte *page) {
    const byte *beginning = hextext;

    // Fill page with erased state
    for (uint16_t i = 0; i < pagesize; i++) {
        page[i] = 0xFF;
    }

    while (1) {
        uint16_t len;
        uint16_t lineaddr;
        byte rectype;
        byte b, cksum = 0;
        char c;

        // Skip blank lines / CRLF leftovers
        do {
            c = pgm_read_byte(hextext++);
            if (c == 0) {
                return nullptr;
            }
        } while (c == '\n' || c == '\r');

        if (c != ':') {
            error(F("No colon?"));
            return nullptr;
        }

        // Byte count
        len = hexToByte(pgm_read_byte(hextext++));
        len = (len << 4) + hexToByte(pgm_read_byte(hextext++));
        cksum = len;

        // Address high
        b = hexToByte(pgm_read_byte(hextext++));
        b = (b << 4) + hexToByte(pgm_read_byte(hextext++));
        cksum += b;
        lineaddr = b << 8;

        // Address low
        b = hexToByte(pgm_read_byte(hextext++));
        b = (b << 4) + hexToByte(pgm_read_byte(hextext++));
        cksum += b;
        lineaddr |= b;

        // Record type
        rectype = hexToByte(pgm_read_byte(hextext++));
        rectype = (rectype << 4) + hexToByte(pgm_read_byte(hextext++));
        cksum += rectype;

        if (rectype == 0x01) {
            // EOF
            // consume checksum
            b = hexToByte(pgm_read_byte(hextext++));
            b = (b << 4) + hexToByte(pgm_read_byte(hextext++));
            cksum += b;

            // optional newline handling
            char eol = pgm_read_byte(hextext);
            if (eol == '\r' || eol == '\n') {
                hextext++;
            }

            if (cksum != 0) {
                error(F("Bad checksum at EOF"));
            }

            return nullptr;
        }

        if (rectype != 0x00) {
            // Unsupported non-data record: skip it cleanly
            for (uint16_t i = 0; i < len; i++) {
                b = hexToByte(pgm_read_byte(hextext++));
                b = (b << 4) + hexToByte(pgm_read_byte(hextext++));
                cksum += b;
            }

            b = hexToByte(pgm_read_byte(hextext++));
            b = (b << 4) + hexToByte(pgm_read_byte(hextext++));
            cksum += b;

            if (cksum != 0) {
                error(F("Bad checksum in non-data record"));
            }

            // eat newline(s)
            char eol = pgm_read_byte(hextext);
            if (eol == '\r') hextext++;
            if (pgm_read_byte(hextext) == '\n') hextext++;

            continue;
        }

        // If this record starts after this page, stop and let the caller
        // retry it on the next page.
        if (lineaddr >= (pageaddr + pagesize)) {
            return beginning;
        }

        // Read data bytes into the correct offset within the page
        for (uint16_t i = 0; i < len; i++) {
            b = hexToByte(pgm_read_byte(hextext++));
            b = (b << 4) + hexToByte(pgm_read_byte(hextext++));
            cksum += b;

            uint16_t absoluteAddr = lineaddr + i;

            if (absoluteAddr >= pageaddr && absoluteAddr < (pageaddr + pagesize)) {
                uint16_t offset = absoluteAddr - pageaddr;
                page[offset] = b;
            }
        }

        // Record checksum
        b = hexToByte(pgm_read_byte(hextext++));
        b = (b << 4) + hexToByte(pgm_read_byte(hextext++));
        cksum += b;

        if (cksum != 0) {
            error(F("Bad checksum in data record"));
        }

        // Eat CR/LF
        if (pgm_read_byte(hextext) == '\r') hextext++;
        if (pgm_read_byte(hextext) == '\n') hextext++;

        // Update "beginning" so next record becomes the retry point
        beginning = hextext;
    }
}


bool verifyImage(const byte *hextext) {
    startProgramMode(FLASH_CLOCKSPEED);  // start at 1MHz speed
    uint16_t len;
    byte b, cksum = 0;

    while (1) {
        uint16_t lineaddr;
        // read one line!
        char c = pgm_read_byte(hextext++);
        if (c == '\n' || c == '\r') {
            continue;
        }
        if (c != ':') {
            Serial.print(c);
            error(F(" No colon?"));
            break;
        }
        len = hexToByte(pgm_read_byte(hextext++));
        len = (len << 4) + hexToByte(pgm_read_byte(hextext++));
        cksum = len;

        b = hexToByte(pgm_read_byte(hextext++));  // record type
        b = (b << 4) + hexToByte(pgm_read_byte(hextext++));
        cksum += b;
        lineaddr = b;
        b = hexToByte(pgm_read_byte(hextext++));  // record type
        b = (b << 4) + hexToByte(pgm_read_byte(hextext++));
        cksum += b;
        lineaddr = (lineaddr << 8) + b;

        b = hexToByte(pgm_read_byte(hextext++));  // record type
        b = (b << 4) + hexToByte(pgm_read_byte(hextext++));
        cksum += b;

        if (b == 0x1) {
            // end record!
            break;
        }

        for (byte i = 0; i < len; i++) {
            // read 'n' bytes
            b = hexToByte(pgm_read_byte(hextext++));
            b = (b << 4) + hexToByte(pgm_read_byte(hextext++));
            cksum += b;

            // verify this byte!
            byte reply;
            if (lineaddr % 2) {  // for 'high' bytes
                reply = isp_transaction(0x28, lineaddr >> 9, lineaddr / 2, 0) & 0xFF;
            } else {  // for 'low' bytes
                reply = isp_transaction(0x20, lineaddr >> 9, lineaddr / 2, 0) & 0xFF;
            }
            if (b != reply) {
                Serial.println();
                Serial.print(F("Verification error at address 0x"));
                Serial.print(lineaddr, HEX);
                Serial.print(F(" Should be 0x"));
                Serial.print(b, HEX);
                Serial.print(F(" not 0x"));
                Serial.println(reply, HEX);
                return false;
            }
            lineaddr++;
        }

        b = hexToByte(pgm_read_byte(hextext++));  // chxsum
        b = (b << 4) + hexToByte(pgm_read_byte(hextext++));
        cksum += b;
        if (cksum != 0) {
            Serial.print(cksum, HEX);
            error(F(" - bad checksum"));
        }
        if (pgm_read_byte(hextext++) != '\n') {
            error(F("No end of line"));
        }
    }

    endProgramMode();
    return true;
}


bool flashWord(uint8_t hilo, uint16_t addr, uint8_t data) {
    if (isp_transaction(0x40 + 8 * hilo, addr >> 8 & 0xFF, addr & 0xFF, data) != addr) {
        return false;
    }
    return true;
}


bool flashPage(byte *pagebuff, uint16_t pageaddr,
               uint16_t pagesize) {
    Serial.print(F("Flashing page "));
    Serial.println(pageaddr, HEX);

    if (millis() - WDDTelapsedTime > 1000) {
        WDDTelapsedTime = millis();
        watchdogReset();  // Feed the watchdog.
        Serial.print(F("WDT reset"));
    }

    startProgramMode(FLASH_CLOCKSPEED);

    for (uint16_t i = 0; i < pagesize / 2; i++) {
        if (!flashWord(LOW, i, pagebuff[2 * i]))
            return false;
        if (!flashWord(HIGH, i, pagebuff[2 * i + 1]))
            return false;
    }

    pageaddr /= 2;

    uint16_t commitreply =
      isp_transaction(0x4C, (pageaddr >> 8) & 0xFF, pageaddr & 0xFF, 0);

    Serial.print(F("  Commit Page: 0x"));
    Serial.print(pageaddr, HEX);
    Serial.print(F(" -> 0x"));
    Serial.println(commitreply, HEX);
    if (commitreply != pageaddr)
        return false;

    busyWait();
    endProgramMode();
    return true;
}


void busyWait(void) {
    byte busybit;
    do {
        busybit = isp_transaction(0xF0, 0x0, 0x0, 0x0);
    } while (busybit & 0x01);
}


uint32_t isp_transaction(uint8_t a, uint8_t b, uint8_t c,
                         uint8_t d) {
    uint8_t l, n, m, o;
    (void)o;  // avoid unused var warning
    (void)m;  // avoid unused var warning
    (void)n;  // avoid unused var warning
    o = transfer(a);
    n = transfer(b);
    // if (n != a) error = -1;
    m = transfer(c);
    l = transfer(d);
    return ((m << 8) + l);
}


uint8_t transfer(uint8_t out) {
    return SPI.transfer(out);
}


byte hexToByte(byte h) {
    if (h >= '0' && h <= '9')
        return (h - '0');
    if (h >= 'A' && h <= 'F')
        return ((h - 'A') + 10);
    if (h >= 'a' && h <= 'f')
        return ((h - 'a') + 10);
    Serial.print("Read odd char 0x");
    Serial.print(h, HEX);
    Serial.println();
    error(F("Bad hex digit!"));
    return -1;
}


void error(const char *string) {
    Serial.println(string);
    while (1) {
    }
}


void error(const __FlashStringHelper *string) {
    Serial.println(string);
    while (1) {
    }
}
