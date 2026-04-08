
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
:100000000C94DD000C94EC060C94C5060C949E0632
:100010000C9477060C9405010C9405010C94500681
:100020000C9405010C9405010C945B030C94C4031F
:100030000C9405010C9405010C9405010C94050128
:100040000C9405010C9405010C9405010C94050118
:100050000C9405010C9405010C9405010C94060602
:100060000C9414070C9405010C9405010C940501E3
:100070000C9405010C9413070C9405010C940501D4
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
:1001B0007300574152424C00370711241FBECFEF46
:1001C000DAE0DEBFCDBF11E0A0E0B1E0E0E9F6E1AA
:1001D00002C005900D92AC34B107D9F721E0ACE430
:1001E000B1E001C01D92AB3EB207E1F710E0CDEDEA
:1001F000D0E004C02197FE010E94400BCC3DD10706
:10020000C9F70E944C070C94460B0C940000909187
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
:10064000CF90BF90AF909F908F9008958FEF8EBD09
:1006500008950895823108F08251E82FF0E0E455C2
:10066000FF4FE49190917B008E2F880F880F80724E
:100670009F7D892B80937B00E770E064E0937C0092
:10068000F89483B7817F826083BF83B7816083BF23
:1006900080917A00886480937A007894889583B7F3
:1006A0008E7F83BF80917A0086FDFCCF8091780099
:1006B0009091790008951F920F920FB60F92112416
:1006C0008F939F938091E1009091E100937F9093AD
:1006D000E10083FF0FC01092E90091E09093EB00DE
:1006E0001092EC0092E39093ED001092EA0198E0F2
:1006F0009093F00082FF1CC093E09093E9009091EA
:10070000F200992319F09AE39093E80090918B01FD
:10071000992329F090918B01915090938B019091A6
:10072000E901992329F09091E90191509093E90111
:1007300084FF18C08091E2008E7E81608093E20089
:100740008091E1008F7E8093E10080918C018E7E0C
:10075000806180938C019F918F910F900FBE0F90BD
:100760001F90189580FFF7CF8091E2008E7E806108
:100770008093E2008091E1008E7E8093E100809181
:100780008C018E7E8160E5CF1F920F920FB60F9283
:100790001124CF92DF92EF92FF920F931F932F932A
:1007A0003F934F935F936F937F938F939F93AF93F9
:1007B000BF93EF93FF93CF93DF93CDB7DEB76C97E3
:1007C000DEBFCDBF1092E9008091E80083FF2FC00B
:1007D00089E0FE017596EE2EDF2E815029F0909172
:1007E000F10090833196F9CF84E68093E90182EF9E
:1007F0008093E8008D8987FF39C09091E80090FFD1
:10080000FCCF982F907609F033C19E894F89588D7F
:100810002F89F88C911131C0803861F580918A015F
:100820008093F1001092F1008EEF8093E8006C96B7
:100830000FB6F894DEBF0FBECDBFDF91CF91FF9111
:10084000EF91BF91AF919F918F917F916F915F9148
:100850004F913F912F911F910F91FF90EF90DF905B
:10086000CF900F900FBE0F901F9018959EEF909312
:10087000E800C7CF1092F100D5CF913059F4811123
:10088000D3CF4130510581F680918A018D7F8093CD
:100890008A01CACF933049F48111C6CF4130510546
:1008A00019F680918A018260F2CF953041F48091EF
:1008B000E80080FFFCCF20682093E300B5CF96309E
:1008C00009F0A8C00B8D1C8D22E01092E900109257
:1008D000880110928701F2122EC0109286011092A8
:1008E00085010E9438021F8299E09983FA8291E083
:1008F0009E8390EA98879AE09987209187013091AA
:100900008801275F3F4F3C832B838D831092E90042
:10091000109288011092870110938601009385013F
:1009200049E050E0BE016F5F7F4F80E00E941202FD
:100930000E94380279CF10938601009385010E94AE
:100940004201DC0112960D911C910115110509F46B
:1009500057C1D801ED91FC910480F581E02D6E2DF9
:100960007D2DC8010995009709F045C1F801008562
:100970001185EBCFF3E0FF120EC08F89882309F4B5
:1009800040C0823061F440E865E082EB91E00E9473
:10099000DF01811149CF81E28093EB0048CF8130A4
:1009A00029F440E86AE187E991E0F1CF833099F7D3
:1009B0000E944201DC011296ED90FC908E010F5FC7
:1009C0001F4F6801E114F10479F0D701ED91FC911A
:1009D0000680F781E02DB801C7010995080F111DA8
:1009E000F701E084F184EECFD8011C92F60101906A
:1009F0000020E9F73197BF016C197D0940E0C6017D
:100A0000C6CF61E871E0FB01449150E080E80E94AC
:100A100012020ACF973009F4BECF983021F481E05A
:100A20008093F10001CF993009F0FECE837009F078
:100A3000B2CFE3E3F1E081E031E096E321912223BC
:100A400071F08093E9003093EB00DF0111972C9156
:100A50002093EC009093ED008F5F873079F78EE75D
:100A60008093EA001092EA008F898093EA01DCCE3D
:100A70008B8D9C8D1092E900109288011092870155
:100A80009093860180938501898D81119AC08E890A
:100A90009D89913A49F4813209F07DCF47E050E0D9
:100AA0006AE271E080E0B3CF913209F074CF833213
:100AB00069F48F89988DB0E0A0E08093260190932F
:100AC0002701A0932801B0932901AECE8032A9F46A
:100AD0008091E80082FFFCCF88E0EAE2F1E08150FB
:100AE00029F09091F10090833196F9CF84E68093BC
:100AF000E9018BEF8093E80097CE823209F094CE23
:100B00008F8980933101EEEFFFE7859194918B3FC0
:100B10009C4D51F1E0E0F8E080912A0190912B0189
:100B2000A0912C01B0912D01803B9440A105B1050D
:100B3000F1F48091310180FD1AC0EE3F8AE0F807A0
:100B400089F587E797E791838083809160008093A0
:100B5000890188E19BE00FB6F894A8958093600026
:100B60000FBE9093600060CEEEEFFAE0D5CF8081AB
:100B700091818737974709F057CEA89580916000FB
:100B80008861809360008091890180936000EE3FCE
:100B90002AE0F20789F08091FE0A9091FF0A918382
:100BA000808342CE808191818737980751F290935C
:100BB000FF0A8093FE0AC5CF1092FF0A1092FE0A28
:100BC00033CE0E944201DC0112960D911C91011559
:100BD000110509F4E0CED801ED91FC910190F0816E
:100BE000E02D6E2D7D2DC801099581111DCEF801D6
:100BF00000851185ECCF181619060CF415CECBCE56
:100C0000F1E0FF12B7CE65E871E0FDCE1F920F92C2
:100C10000FB60F9211242F933F938F939F93AF930F
:100C2000BF9380917D0190917E01A0917F01B09151
:100C3000800130917C0126E0230F2D3758F5029674
:100C4000A11DB11D20937C0180937D0190937E01B5
:100C5000A0937F01B09380018091810190918201E6
:100C6000A0918301B09184010196A11DB11D8093D3
:100C7000810190938201A0938301B0938401BF917D
:100C8000AF919F918F913F912F910F900FBE0F9039
:100C90001F90189529E8230F0396A11DB11DD2CFEF
:100CA0001F920F920FB60F9211242F933F934F93E1
:100CB0005F936F937F938F939F93AF93BF93EF93C4
:100CC000FF93E0910801F09109010995FF91EF91DF
:100CD000BF91AF919F918F917F916F915F914F9154
:100CE0003F912F910F900FBE0F901F9018951F925C
:100CF0000F920FB60F9211242F933F934F935F9350
:100D00006F937F938F939F93AF93BF93EF93FF93D3
:100D1000E0910601F09107010995FF91EF91BF91D4
:100D2000AF919F918F917F916F915F914F913F9183
:100D30002F910F900FBE0F901F9018951F920F923A
:100D40000FB60F9211242F933F934F935F936F939E
:100D50007F938F939F93AF93BF93EF93FF93E09114
:100D60000401F09105010995FF91EF91BF91AF91B9
:100D70009F918F917F916F915F914F913F912F91B3
:100D80000F900FBE0F901F9018951F920F920FB6E5
:100D90000F9211242F933F934F935F936F937F9301
:100DA0008F939F93AF93BF93EF93FF93E0910201D3
:100DB000F09103010995FF91EF91BF91AF919F9140
:100DC0008F917F916F915F914F913F912F910F90F4
:100DD0000FBE0F901F9018951F920F920FB60F9293
:100DE00011242F933F934F935F936F937F938F9330
:100DF0009F93AF93BF93EF93FF93E0910001F09126
:100E000001010995FF91EF91BF91AF919F918F9152
:100E10007F916F915F914F913F912F910F900FBEF6
:100E20000F901F90189518951F920F920FB60F9262
:100E300011248F93EF93FF93EEB5EC3068F4F0E05C
:100E4000E45BFE4F80818EBDFF91EF918F910F90FB
:100E50000FBE0F901F901895E43119F410922501E0
:100E6000F3CFE53189F781E080932501EDCFEDE8FF
:100E7000F1E01382128288EE93E0A0E0B0E0848378
:100E80009583A683B7838DE391E0918380838FEF71
:100E90009FEF958784870895789484B5826084BD98
:100EA00084B5816084BD85B5826085BD85B581606E
:100EB00085BD80916E00816080936E0010928100EC
:100EC00080918100826080938100809181008160A7
:100ED0008093810080918000816080938000809168
:100EE0009100826080939100809191008160809355
:100EF0009100809190008160809390008091C1006A
:100F000084608093C1008091C10082608093C100A1
:100F10008091C10081608093C1008091C300816095
:100F20008093C3008091C00082608093C000809154
:100F3000C20081608093C20080917A0084608093B7
:100F40007A0080917A00826080937A0080917A00A2
:100F50008E7F80937A0080917A00806880937A00F7
:100F60001092EA0110928A0110928C018091D700B0
:100F700081608093D70080EA8093D80089B58F7E06
:100F800089BD89B5826089BD09B400FEFDCF0E948C
:100F90008E028091D8008F7C80618093D8008091F0
:100FA000E000807F8093E0008091E1008E7E80935E
:100FB000E1008DE08093E2008091D8008062809310
:100FC000D80089B58D7F89BD8091D8008F778093B7
:100FD000D8008091650080688093650080916400EE
:100FE00082608093640080916400806280936400DA
:100FF00080916400886080936400809164008064C4
:101000008093640080916500886080936500809182
:1010100065008160809365008FEF80937E0083EF91
:1010200080937D0090E080E0FC01EE0FFF1FED5EFD
:10103000FE4F208131814FB7F894F901FF272291AB
:10104000232B20834FBF01968930910569F780914A
:101050007A00887F80937A0084E880937A0080E425
:1010600080937C008CB580648CBDEFE0F1E024912E
:10107000E0EFF0E08491882399F090E0880F991FC9
:10108000FC01EA53FF4FA591B491FC01E854FF4FD6
:10109000859194918FB7F894EC91E22BEC938FBFEC
:1010A0008CB580688CBDE4E0F1E02491E5EEF0E0E1
:1010B00084918823C1F090E0880F991FFC01EA53C6
:1010C000FF4FC591D491FC01E854FF4FA591B49115
:1010D0009FB7F8943881822F809583238883EC9181
:1010E000E22BEC939FBF86E293E090930101809303
:1010F0000001809169008C7F826080936900E89A8A
:1011000091E3892E91E0992E02E111E023EFE22E86
:1011100020E0F22E37E5C32E31E0D32E40E0A42E9E
:1011200040E0B42E83B7817F8C6083BFF8948091B8
:10113000640081608093640083B7816083BF78948A
:10114000889583B78E7F83BF809164008E7F809364
:101150006400F4019491F8012491F70184918823AB
:10116000C1F39923E9F091509F30D0F4E92FF0E0DA
:10117000E454F74F0C94400B1B0B1F0BCB08130BC5
:10118000170BD008D008D008220B280B2C0B300BE3
:10119000360BD0083A0B909180009F779093800097
:1011A000E82FF0E0EE0FFF1FEC52FF4FA591B49136
:1011B0008C91282309F4CDCF80912501882361F0FB
:1011C00080911301909114012FB7F894FC01FF272F
:1011D0008081892B80832FBF80910A010E942A037E
:1011E00080910B010E942A0390936D0180936C0102
:1011F00080910A010E942A0320916A0130916B01BB
:10120000821B930B90935901809358018091250183
:10121000811104C0109259011092580120911301BC
:10122000309114019FB7F894F901FF272081832F93
:101230008095822380839FBF809115019091160134
:101240002FB7F894FC01FF278081892B80832FBF63
:1012500080910C010E942A0390936F0180936E018C
:1012600080910B010E942A0320916C0130916D0145
:10127000821B930B90935B0180935A01209115017F
:10128000309116019FB7F894F901FF272081832F31
:101290008095822380839FBF8091170190911801D0
:1012A0002FB7F894FC01FF278081892B80832FBF03
:1012B00080910D010E942A03909371018093700127
:1012C00080910C010E942A0320916E0130916F01E0
:1012D000821B930B90935D0180935C012091170119
:1012E000309118019FB7F894F901FF272081832FCF
:1012F0008095822380839FBF8091190190911A016C
:101300002FB7F894FC01FF278081892B80832FBFA2
:1013100080910E010E942A039093730180937201C1
:1013200080910D010E942A0320917001309171017A
:10133000821B930B90935F0180935E0120911901B2
:1013400030911A019FB7F894F901FF272081832F6C
:101350008095822380839FBF80911B0190911C0107
:101360002FB7F894FC01FF278081892B80832FBF42
:1013700080910F010E942A0390937501809374015C
:1013800080910E010E942A03209172013091730115
:10139000821B930B909361018093600120911B014C
:1013A00030911C019FB7F894F901FF272081832F0A
:1013B0008095822380839FBF80911D0190911E01A3
:1013C0002FB7F894FC01FF278081892B80832FBFE2
:1013D000809110010E942A039093770180937601F7
:1013E00080910F010E942A032091740130917501B0
:1013F000821B930B909363018093620120911D01E6
:1014000030911E019FB7F894F901FF272081832FA7
:101410008095822380839FBF80911F01909120013E
:101420002FB7F894FC01FF278081892B80832FBF81
:10143000809111010E942A03909379018093780191
:10144000809110010E942A0320917601309177014A
:10145000821B930B909365018093640120911F017F
:10146000309120019FB7F894F901FF272081832F45
:101470008095822380839FBF8091210190912201DA
:101480002FB7F894FC01FF278081892B80832FBF21
:10149000809112010E942A0390937B0180937A012C
:1014A000809111010E942A032091780130917901E5
:1014B000821B930B90936701809366012091210119
:1014C000309122019FB7F894F901FF272081832FE3
:1014D0008095822380839FBF809123019091240176
:1014E0002FB7F894FC01FF278081892B80832FBFC1
:1014F00080910A010E942A0390936B0180936A01F4
:10150000809112010E942A0320917A0130917B017F
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
:10161000892B809357017894A114B10409F482CDE9
:101620000E9400007FCD909180009F7DB7CD90916A
:101630008000977FB3CD94B59F7794BDB1CD94B51D
:101640009F7DFBCF909190009F7790939000A8CDC5
:10165000909190009F7DF9CF90919000977FF5CF6A
:101660009091C0009F779093C0009ACD9091C00058
:101670009F7DF9CF9091C200977F9093C20090CD4B
:10168000EE0FFF1F0590F491E02D0994F894FFCF21
:1016900029032903290329032903000908070B0645
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
