
// Reprogram the ATmega32u4 if necessary.
// Modified from the Adafruit_AVRProg library.
// Green LED will blink 3 times to indicate success.
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
:100000000C94BB000C94C6040C949F040C947804CC
:100010000C9451040C94D8000C94D8000C942A042D
:100020000C94D8000C94D8000C94CE010C94370298
:100030000C94D8000C94D8000C94D8000C94D800E0
:100040000C94D8000C94D8000C94D8000C94D800D0
:100050000C94D8000C94D8000C94D8000C94E003B5
:100060000C94EE040C94D8000C94D8000C94D80096
:100070000C94D8000C94ED040C94D8000C94D80087
:100080000C94D8000C94D8000C94D8000C94D80090
:100090000C94D8000C94D8000C94D8000C94D80080
:1000A0000C94D8000C94D8000C94D80007060504D2
:1000B0000100080A0B0C0D090000000025002800B3
:1000C0002B002E00310000000000240027002A0031
:1000D0002D003000000000002300260029002C0025
:1000E0002F000404040404030405020202020403B2
:1000F00002020202060606060606040402020204C2
:100100000404080201104080401020408040800814
:100110000204018040201002011080102040402085
:100120000000000200090F0000030401000C0000A1
:1001300000000000000000000000000000000004BB
:100140000309041201000200000040D80487EE00F9
:1001500001010203014D6F77727920537472696E49
:1001600067656420496E737472756D656E74730093
:10017000574152424C0011241FBECFEFDAE0DEBFE0
:10018000CDBF11E0A0E0B1E0E8ECF1E102C00590E4
:100190000D92A632B107D9F721E0A6E2B1E001C085
:1001A0001D92A437B207E1F70E9404050C94E208FF
:1001B0000C94000080916C0181110BC010926801B9
:1001C00081E08093690110926B0110926A01809323
:1001D0006C0188E691E008950F931F93CF93DF930E
:1001E0001F92CDB7DEB719820E94DA00DC011296A9
:1001F0000D911C910115110589F0D801ED91FC912B
:100200000280F381E02DBE016F5F7F4FC801099529
:1002100097FD04C0F80100851185ECCF89810F900E
:10022000DF91CF911F910F910895409161015091FD
:10023000620120915F013091600142175307B4F4CD
:100240009091E8009570E1F39091E80092FD19C05B
:100250008093F100809161019091620101968F730A
:100260009927892B19F48EEF8093E8008091610122
:10027000909162010196909362018093610181E007
:10028000089580E00895EF92FF920F931F93CF930C
:10029000DF93F82E192FE62E042F81E0860F880FAA
:1002A0000E94150183E00E941501CF2DD12FEC0E85
:1002B000FD2EF11CCE15DF05B9F007FF13C0FE01BE
:1002C00084910E941501182F80E00E94150181235E
:1002D00021968111EFCFDF91CF911F910F91FF9068
:1002E000EF9008958881EDCF81E0F5CF3FB7F89486
:1002F00080915B0190915C01A0915D01B0915E01E4
:1003000026B5A89B05C02F3F19F00196A11DB11D70
:100310003FBFBA2FA92F982F8827BC01CD01620FAC
:10032000711D811D911D43E0660F771F881F991F66
:100330004A95D1F7089508950895823108F08251C1
:10034000E82FF0E0E455FF4FE49190917B008E2F71
:10035000880F880F80729F7D892B80937B00E770C8
:10036000E064E0937C00F89483B7817F826083BF70
:1003700083B7816083BF80917A00886480937A001C
:100380007894889583B78E7F83BF80917A0086FDAD
:10039000FCCF809178009091790008951F920F9280
:1003A0000FB60F9211248F939F938091E10090914B
:1003B000E100937F9093E10083FF0FC01092E9006A
:1003C00091E09093EB001092EC0092E39093ED009B
:1003D0001092670198E09093F00082FF1CC093E0B8
:1003E0009093E9009091F200992319F09AE3909389
:1003F000E80090916601992329F09091660191504F
:100400009093660190916501992329F0909165017F
:1004100091509093650184FF18C08091E2008E7E18
:1004200081608093E2008091E1008F7E8093E10003
:10043000809164018E7E8061809364019F918F9191
:100440000F900FBE0F901F90189580FFF7CF8091EF
:10045000E2008E7E80618093E2008091E1008E7EDA
:100460008093E100809164018E7E8160E5CF1F92D0
:100470000F920FB60F921124CF92DF92EF92FF925C
:100480000F931F932F933F934F935F936F937F939C
:100490008F939F93AF93BF93EF93FF93CF93DF938C
:1004A000CDB7DEB76C97DEBFCDBF1092E90080916B
:1004B000E80083FF2FC089E0FE017596EE2EDF2E47
:1004C000815029F09091F10090833196F9CF84E624
:1004D0008093650182EF8093E8008D8987FF39C0A2
:1004E0009091E80090FFFCCF982F907609F035C1ED
:1004F0009E894F89588D2F89F88C911131C0803891
:1005000061F5809163018093F1001092F1008EEF0C
:100510008093E8006C960FB6F894DEBF0FBECDBF97
:10052000DF91CF91FF91EF91BF91AF919F918F910B
:100530007F916F915F914F913F912F911F910F91FB
:10054000FF90EF90DF90CF900F900FBE0F901F9015
:1005500018959EEF9093E800C7CF1092F100D5CF89
:10056000913059F48111D3CF4130510581F68091FA
:1005700063018D7F80936301CACF933049F4811169
:10058000C6CF4130510519F6809163018260F2CFE8
:10059000953041F48091E80080FFFCCF20682093E3
:1005A000E300B5CF963009F0C0C00B8D1C8D82E002
:1005B0001092E9001092620110926101F81237C0A6
:1005C0001092600110925F010E94EC001F8299E07E
:1005D0009983FA8291E09E8390EA98879AE09987BE
:1005E0002091610130916201275F3F4F3C832B8353
:1005F0008D831092E90010926201109261011093B4
:10060000600100935F018E010F5F1F4F7E01AAE022
:10061000EA0EF11C0E151F0539F0F80181918F01CA
:100620000E9415018111F6CF0E94EC0070CF10934B
:10063000600100935F010E94DA00DC0112960D91C7
:100640001C910115110509F4B3C0D801ED91FC917D
:100650000480F581E02D6E2D7D2DC8010995009750
:1006600009F0A1C0F80100851185EBCFF3E0FF127E
:100670000EC08F89882309F440C0823061F440E8BD
:1006800065E080E791E00E944301811140CF81E263
:100690008093EB003FCF813029F440E86AE185E5A3
:1006A00091E0F1CF833099F70E94DA00DC011296D5
:1006B000ED90FC908E010F5F1F4F6801E114F10473
:1006C00079F0D701ED91FC910680F781E02DB8011A
:1006D000C7010995080F111DF701E084F184EECFE1
:1006E000D8011C92F60101900020E9F73197BF0173
:1006F0006C197D0940E0C601C6CF9FE3E92E91E069
:10070000F92EF701C490D12C10E000E00C151D0566
:1007100009F4FDCEF701E00FF11F84910E9415014D
:100720000F5F1F4F8111F2CFF2CE973009F4AFCF98
:10073000983021F481E08093F100E9CE993009F0FE
:10074000E6CE837009F0A3CF8EE78093EA00109283
:10075000EA008F8980936701DACE8B8D9C8D109291
:10076000E900109262011092610190936001809300
:100770005F010E94DA00DC0112960D911C910115B7
:10078000110509F484CFD801ED91FC910190F0811D
:10079000E02D6E2D7D2DC80109958111B8CEF8018F
:1007A00000851185ECCF181619060CF4B0CE6FCF6A
:1007B000F1E0FF125BCF83E4E82E81E0F82EA1CFB9
:1007C0001F920F920FB60F9211242F933F938F9386
:1007D0009F93AF93BF938091570190915801A0913F
:1007E0005901B0915A013091560126E0230F2D375F
:1007F00058F50296A11DB11D209356018093570113
:1008000090935801A0935901B0935A0180915B01D4
:1008100090915C01A0915D01B0915E010196A11DD6
:10082000B11D80935B0190935C01A0935D01B09337
:100830005E01BF91AF919F918F913F912F910F904A
:100840000FBE0F901F90189529E8230F0396A11D46
:10085000B11DD2CF1F920F920FB60F9211242F937A
:100860003F934F935F936F937F938F939F93AF9338
:10087000BF93EF93FF93E0910801F091090109956F
:10088000FF91EF91BF91AF919F918F917F916F9168
:100890005F914F913F912F910F900FBE0F901F903E
:1008A00018951F920F920FB60F9211242F933F931A
:1008B0004F935F936F937F938F939F93AF93BF9368
:1008C000EF93FF93E0910601F09107010995FF91E5
:1008D000EF91BF91AF919F918F917F916F915F91B8
:1008E0004F913F912F910F900FBE0F901F90189531
:1008F0001F920F920FB60F9211242F933F934F9395
:100900005F936F937F938F939F93AF93BF93EF9377
:10091000FF93E0910401F09105010995FF91EF919A
:10092000BF91AF919F918F917F916F915F914F9107
:100930003F912F910F900FBE0F901F9018951F920F
:100940000F920FB60F9211242F933F934F935F9303
:100950006F937F938F939F93AF93BF93EF93FF9387
:10096000E0910201F09103010995FF91EF91BF9190
:10097000AF919F918F917F916F915F914F913F9137
:100980002F910F900FBE0F901F9018951F920F92EE
:100990000FB60F9211242F933F934F935F936F9352
:1009A0007F938F939F93AF93BF93EF93FF93E091C8
:1009B0000001F09101010995FF91EF91BF91AF9175
:1009C0009F918F917F916F915F914F913F912F9167
:1009D0000F900FBE0F901F90189518951F920F92B1
:1009E0000FB60F9211248F93EF93FF93EEB5F0E0C3
:1009F000EA5DFE4F80818EBDFF91EF918F910F9048
:100A00000FBE0F901F901895789484B5826084BDB6
:100A100084B5816084BD85B5826085BD85B5816002
:100A200085BD80916E00816080936E001092810080
:100A3000809181008260809381008091810081603B
:100A400080938100809180008160809380008091FC
:100A500091008260809391008091910081608093E9
:100A60009100809190008160809390008091C100FE
:100A700084608093C1008091C10082608093C10036
:100A80008091C10081608093C1008091C30081602A
:100A90008093C3008091C00082608093C0008091E9
:100AA000C20081608093C20080917A00846080934C
:100AB0007A0080917A00826080937A0080917A0037
:100AC0008E7F80937A0080917A00806880937A008C
:100AD0001092670110926301109264018091D70017
:100AE00081608093D70080EA8093D80089B58F7E9B
:100AF00089BD89B5826089BD09B400FEFDCF0E9421
:100B000076016B017C010E9476016C197D098E09CA
:100B10009F09683E734081059105A8F38091D80034
:100B20008F7C80618093D8008091E000807F8093EB
:100B3000E0008091E1008E7E8093E1008DE0809363
:100B4000E2008091D80080628093D80089B58D7FC3
:100B500089BD8091D8008F778093D80080916500FF
:100B60008068809365008091640082608093640057
:100B7000809164008062809364008091640088604A
:100B8000809364008091640080648093640080910D
:100B90006500886080936500809165008160809326
:100BA00065008FEF80937E0083EF80937D0090E05F
:100BB00080E0FC01EE0FFF1FED5EFE4F20813181D2
:100BC0004FB7F894F901FF272291232B20834FBFC1
:100BD00001968930910569F780917A00887F80932A
:100BE0007A0084E880937A0080E480937C008CB55E
:100BF00080648CBDEFE0F1E02491E0EFF0E08491BF
:100C0000882399F090E0880F991FFC01EA53FF4F69
:100C1000A591B491FC01E854FF4F859194918FB751
:100C2000F894EC91E22BEC938FBF8CB580688CBD6F
:100C3000E4E0F1E02491E5EEF0E084918823C1F056
:100C400090E0880F991FFC01EA53FF4FC591D491A2
:100C5000FC01E854FF4FA591B4919FB7F8943881F7
:100C6000822F809583238883EC91E22BEC939FBFA6
:100C70008BE991E0909301018093000180916900DC
:100C80008C7F826080936900E89A91E3892E91E0DD
:100C9000992E02E111E023EFE22E20E0F22E34E45F
:100CA000C32E31E0D32E40E0A42E40E0B42E83B713
:100CB000817F8C6083BFF894809164008160809311
:100CC000640083B7816083BF7894889583B78E7FF3
:100CD00083BF809164008E7F80936400F4019491BF
:100CE000F8012491F70184918823C1F39923E9F055
:100CF00091509F30D0F4E92FF0E0EF57F94F0C946A
:100D0000DC08B708BB089006AF08B308950695063F
:100D10009506BE08C408C808CC08D2089506D608AF
:100D2000909180009F7790938000E82FF0E0EE0F85
:100D3000FF1FEC52FF4FA591B4918C91282309F429
:100D4000CDCF80911301909114012FB7F894FC013D
:100D5000FF278081892B80832FBF80910A010E9409
:100D60009D0180910B010E949D019093470180930A
:100D7000460180910A010E949D0120914401309119
:100D80004501821B930B9093330180933201209194
:100D90001301309114019FB7F894F901FF272081C6
:100DA000832F8095822380839FBF8091150190912E
:100DB00016012FB7F894FC01FF278081892B8083CF
:100DC0002FBF80910C010E949D0190934901809357
:100DD000480180910B010E949D01209146013091B4
:100DE0004701821B930B909335018093340120912E
:100DF0001501309116019FB7F894F901FF27208162
:100E0000832F8095822380839FBF809117019091CB
:100E100018012FB7F894FC01FF278081892B80836C
:100E20002FBF80910D010E949D0190934B018093F3
:100E30004A0180910C010E949D012091480130914E
:100E40004901821B930B90933701809336012091C7
:100E50001701309118019FB7F894F901FF272081FD
:100E6000832F8095822380839FBF80911901909169
:100E70001A012FB7F894FC01FF278081892B80830A
:100E80002FBF80910E010E949D0190934D01809390
:100E90004C0180910D010E949D0120914A013091E9
:100EA0004B01821B930B9093390180933801209161
:100EB000190130911A019FB7F894F901FF27208199
:100EC000832F8095822380839FBF80911B01909107
:100ED0001C012FB7F894FC01FF278081892B8083A8
:100EE0002FBF80910F010E949D0190934F0180932D
:100EF0004E0180910E010E949D0120914C01309184
:100F00004D01821B930B90933B0180933A012091FA
:100F10001B0130911C019FB7F894F901FF27208134
:100F2000832F8095822380839FBF80911D019091A4
:100F30001E012FB7F894FC01FF278081892B808345
:100F40002FBF809110010E949D01909351018093C9
:100F5000500180910F010E949D0120914E0130911E
:100F60004F01821B930B90933D0180933C01209194
:100F70001D0130911E019FB7F894F901FF272081D0
:100F8000832F8095822380839FBF80911F01909142
:100F900020012FB7F894FC01FF278081892B8083E3
:100FA0002FBF809111010E949D0190935301809366
:100FB0005201809110010E949D01209150013091B9
:100FC0005101821B930B90933F0180933E0120912E
:100FD0001F01309120019FB7F894F901FF2720816C
:100FE000832F8095822380839FBF809121019091E0
:100FF00022012FB7F894FC01FF278081892B808381
:101000002FBF809112010E949D0190935501809302
:101010005401809111010E949D0120915201309153
:101020005301821B930B90934101809340012091C7
:101030002101309122019FB7F894F901FF27208107
:10104000832F8095822380839FBF8091230190917D
:1010500024012FB7F894FC01FF278081892B80831E
:101060002FBF80910A010E949D01909345018093BA
:101070004401809112010E949D0120915401309100
:101080005501821B930B9093430180934201209161
:101090002301309124019FB7F894F901FF272081A3
:1010A000832F8095822380839FBFA2E3B1E0FD015F
:1010B0008191919197FF05C0CF010297EC011982B0
:1010C0001882CE16DF06A1F7F894E2E3F1E090E093
:1010D00080E0408132969C012A5D3E4FE9014883C1
:1010E000019689309105A9F710922F0110923001D5
:1010F0001092310180E090912F0111962C9111975F
:101100001296922B90932F01833031F090912F0102
:10111000990F990F90932F018F5F843061F7EAE365
:10112000F1E09091300121813296922B9093300121
:10113000873031F090913001990F990F90933001E1
:101140008F5F883071F780914301809331017894EB
:10115000A114B10409F4ABCD0E940000A8CD909178
:1011600080009F7DE0CD90918000977FDCCD94B58D
:101170009F7794BDDACD94B59F7DFBCF9091900081
:101180009F7790939000D1CD909190009F7DF9CF63
:1011900090919000977FF5CF9091C0009F779093AA
:1011A000C000C3CD9091C0009F7DF9CF9091C20047
:1011B000977F9093C200B9CDEE0FFF1F0590F49179
:0811C000E02D0994F894FFCF23
:1011C8009C019C019C019C019C01000908070B06DD
:1011D8000402012E402880284025802B042B08314A
:0611E8000131102540005A
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

    for (byte i = 0; i < 4; i++) {  // Indicate success.
        analogWrite(GREEN_LED, 1023);
        delay(200);
        analogWrite(GREEN_LED, 0);
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
        analogWrite(GREEN_LED, 1023);
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
        analogWrite(GREEN_LED, 0);
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
                          uint16_t pagesize, byte *page) {
    uint16_t len;
    uint16_t page_idx = 0;
    const byte *beginning = hextext;

    byte b, cksum = 0;

    // 'empty' the page by filling it with 0xFF's
    for (uint16_t i = 0; i < pagesize; i++)
        page[i] = 0xFF;

    while (1) {
        uint16_t lineaddr;
        char c;

        // read one line!
        c = pgm_read_byte(hextext++);
        if (c == '\n' || c == '\r') {
            continue;
        }
        if (c != ':') {
            error(F(" No colon?"));
            break;
        }
        // Read the byte count into 'len'
        len = hexToByte(pgm_read_byte(hextext++));
        len = (len << 4) + hexToByte(pgm_read_byte(hextext++));
        cksum = len;
        // Serial.print(len);

        // read high address byte
        b = hexToByte(pgm_read_byte(hextext++));
        b = (b << 4) + hexToByte(pgm_read_byte(hextext++));
        cksum += b;
        lineaddr = b;

        // read low address byte
        b = hexToByte(pgm_read_byte(hextext++));
        b = (b << 4) + hexToByte(pgm_read_byte(hextext++));
        cksum += b;
        lineaddr = (lineaddr << 8) + b;

        if (lineaddr >= (pageaddr + pagesize)) {
            return beginning;
        }

        b = hexToByte(pgm_read_byte(hextext++));  // record type
        b = (b << 4) + hexToByte(pgm_read_byte(hextext++));
        cksum += b;

        if (b == 0x1) {
            // end record, return nullptr to indicate we're done
            hextext = nullptr;
            break;
        }

        for (byte i = 0; i < len; i++) {
            // read 'n' bytes
            b = hexToByte(pgm_read_byte(hextext++));
            b = (b << 4) + hexToByte(pgm_read_byte(hextext++));
            cksum += b;
            page[page_idx] = b;
            page_idx++;

            if (page_idx > pagesize) {
                error("Too much code!");
                break;
            }
        }
        b = hexToByte(pgm_read_byte(hextext++));  // chxsum
        b = (b << 4) + hexToByte(pgm_read_byte(hextext++));
        cksum += b;
        if (cksum != 0) {
            error(F("Bad checksum: "));
            Serial.print(cksum, HEX);
        }
        if (pgm_read_byte(hextext++) != '\n') {
            error(F("No end of line"));
            break;
        }

        if (page_idx == pagesize)
            break;
    }

    return hextext;
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
    analogWrite(GREEN_LED, 1023);
    while (1) {
    }
}


void error(const __FlashStringHelper *string) {
    Serial.println(string);
    analogWrite(GREEN_LED, 1023);
    while (1) {
    }
}
