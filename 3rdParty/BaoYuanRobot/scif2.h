#ifndef SC2_H
#define SC2_H

#include "scif2_define.h"

#ifdef __QT
    #include <QtCore/qglobal.h>
    #define __CLASS
    #if defined(SC2_LIBRARY)
    #  define SC2SHARED_EXPORT Q_DECL_EXPORT
    #else
    #  define SC2SHARED_EXPORT Q_DECL_IMPORT
    #endif
    #define IMEXPORT
    #define CALBY
#else
    #if defined(__CLASS)
        #if defined(SC2_LIBRARY)
            #define SC2SHARED_EXPORT __declspec(dllexport)
            #define IMEXPORT
            #define CALBY
        #else
            #define SC2SHARED_EXPORT __declspec(dllimport)
            #define IMEXPORT
            #define CALBY
        #endif
    #else
        #ifdef __BCB
            #if defined(SC2_LIBRARY)
                #define IMEXPORT __declspec(dllexport)
                #define CALBY      __stdcall
            #else
                #define IMEXPORT __declspec(dllimport)
                #define CALBY    __stdcall
            #endif
        #endif
        #ifdef __GCC
            #define IMEXPORT
            #define CALBY 
        #endif
        #ifdef __VC
            #if defined(SC2_LIBRARY)
                #define IMEXPORT __declspec(dllexport)
                #define CALBY    __cdecl
            #else
                #define IMEXPORT __declspec(dllimport)
                #define CALBY    __cdecl
            #endif
        #endif
    #endif
#endif

#if defined(__CLASS)
class SC2SHARED_EXPORT SC2
{
public:
    IMEXPORT CALBY SC2();
#else
    extern "C" {
#endif
    //=================== 通訊執行函式，應該每10~100ms呼叫一次 ======================
    IMEXPORT int  CALBY MainProcess();
    //=================== 函式庫初始化 ===================
    IMEXPORT int  CALBY LibraryInitial (DLL_USE_SETTING *pUseSetting, int MakerID, char* EncString);
    IMEXPORT void CALBY LibraryDestroy ();
    IMEXPORT void CALBY LibrarySetDebug (int level);   //level : 0不顯示   1有錯誤才顯示   2通訊內容全部顯示(除錯用，通訊速度變成每秒一次)
    //--==========================取得內部資訊 =================================
    IMEXPORT int   CALBY GetLibraryMsg (int id );                   //取得一般的資訊
    IMEXPORT int   CALBY GetConnectionMsg (int TgrConn, int MsgID);         //取得連線通訊的資訊
    IMEXPORT void  CALBY GetConnectionError (int TgrConn, ERROR_MSG *Msg);  //取得錯誤訊息

    //===================區域網路中偵測主機功能=====================================
    IMEXPORT int  CALBY LocalDetectControllers ();                                    //自動偵測主機功能
    IMEXPORT int  CALBY LocalReadControllerCount ();                                  //讀取取得的控制器資料筆數
    IMEXPORT int  CALBY LocalReadController (int Index, LOCAL_CONTROLLER_INFO *Info); //讀取取得的控制器資料

    //==================================連線功能=================================
    IMEXPORT int  CALBY ConnectSetPwd (int TgrConn, char *Pwd);     //設定連線密碼
    IMEXPORT int  CALBY ConnectLocalList (int TgrConn, int Index);  //與取得的控制器清單中的Index值進行連線
    IMEXPORT int  CALBY ConnectLocalIP (int TgrConn, char* IP);     //直接輸入控制器IP進行連線
    IMEXPORT int  CALBY Disconnect (int TgrConn);                   //中斷連線

    //==================================檔案傳輸功能=================================
    IMEXPORT int  CALBY FtpSetConnection (int TgrConn);                                                     //設定 FTP 索引
    //==================================遠端檔案清單
    IMEXPORT int  CALBY FtpGetFileList (int Folder, char *SubFolder, char *HeadFilter, char *TailFilter);   //取得檔案清單
    IMEXPORT int  CALBY FtpReadFileCount ();                                                                //讀取FTP檔案清單筆數
    IMEXPORT int  CALBY FtpReadFile (int Index, FTP_FILE *file);                                            //讀取FTP檔案名稱
    //==================================檔案傳輸功能=================================
    IMEXPORT int  CALBY FtpMakeDir (int Folder, char *DirName);                                             //建立資料夾
    IMEXPORT int  CALBY FtpUpload1File (int Folder, char *SubFolder, char *Filename, char *LocalFilename);   //上傳檔案
    IMEXPORT int  CALBY FtpDownload1File (int Folder, char *SubFolder, char *Filename, char *LocalFilename); //下載檔案
    IMEXPORT int  CALBY FtpDelete1File (int Folder, char *SubFolder, char *Filename);                        //刪除檔案
    IMEXPORT int  CALBY FtpTransferFileReset ();
    IMEXPORT int  CALBY FtpTransferFileAdd (int Folder, char *SubFolder, char *Filename, char *LocalFilename);
    IMEXPORT int  CALBY FtpUploadFiles ();
    IMEXPORT int  CALBY FtpDownloadFiles ();
    IMEXPORT int  CALBY FtpDeleteFiles ();
    //==================================
    IMEXPORT int  CALBY FtpCheckDone ();                                             //取得執行結果
    IMEXPORT int  CALBY FtpWaitDone (int MaxWaitTime);                               //取得執行結果
    //==================================本地端檔案清單
    IMEXPORT int  CALBY LocalGetFileList (char *Path, char *HeadFilter, char *TailFilter);  //取得本地端檔案清單
    IMEXPORT int  CALBY LocalReadFileCount ();                                              //讀取本地檔案清單筆數
    IMEXPORT int  CALBY LocalReadFile (int Index, FTP_FILE *file);                          //讀取本地檔案名稱
    IMEXPORT int  CALBY LocalDeleteFile (int Index);                                        //刪除本地檔案名稱

    //====================== 由本地記憶體讀取資料
    IMEXPORT int  CALBY memI (int TgrConn, int addr );
    IMEXPORT int  CALBY memO (int TgrConn, int addr );
    IMEXPORT int  CALBY memC (int TgrConn, int addr );
    IMEXPORT int  CALBY memS (int TgrConn, int addr );
    IMEXPORT int  CALBY memA (int TgrConn, int addr );
    IMEXPORT int  CALBY memR (int TgrConn, int addr );
    IMEXPORT int  CALBY memRBit (int TgrConn, int addr, int BitIdx);
    IMEXPORT int  CALBY memRString (int TgrConn, int addr, int RSize, char *Buf );           //由鏡射記憶體中讀取字串
    IMEXPORT double CALBY memF (int TgrConn, int addr );
    IMEXPORT char* CALBY memRPtr (int TgrConn, int addr );
    //----
    /*
    IMEXPORT int  CALBY memTimerSet (int TgrConn, int addr );
    IMEXPORT int  CALBY memTimerVal (int TgrConn, int addr );
    IMEXPORT int  CALBY memTimerTrg (int TgrConn, int addr );
    IMEXPORT int  CALBY memCounterSet (int TgrConn, int addr );
    IMEXPORT int  CALBY memCounterVal (int TgrConn, int addr );
    IMEXPORT int  CALBY memCounterTrg (int TgrConn, int addr );
    */

    //====================== 寫入本地記憶體
    IMEXPORT int  CALBY memSetO (int TgrConn, int addr, char val);
    IMEXPORT int  CALBY memSetC (int TgrConn, int addr, char val);
    IMEXPORT int  CALBY memSetS (int TgrConn, int addr, char val);
    IMEXPORT int  CALBY memSetA (int TgrConn, int addr, char val);
    IMEXPORT int  CALBY memSetR (int TgrConn, int addr, int val);
    IMEXPORT int  CALBY memSetRBit (int TgrConn, int addr, int BitIdx, int val);
    IMEXPORT int  CALBY memSetRString (int TgrConn, int addr, int RSize, char *Buf );           //由鏡射記憶體中讀取字串
    IMEXPORT int  CALBY memSetF (int TgrConn, int addr, double val);


    //============================= 取得通訊處理狀態
    IMEXPORT int       CALBY GetTranState (int pTran);
    IMEXPORT SC_DATA*  CALBY GetDataPointer (int pTran);
    // ========================================================
    //以下函式回傳值若為 0 ，代表指令初拒絕，若不為 0 ，代表是該筆通訊的指標
    //此時，將其帶入 GetTranState 的引數中，取得該筆通訊的狀態，將會是 SC_TRANSACTION_RESET
    //一段時間之後再帶入 GetTranState 的引數中，回傳值若為 SC_TRANSACTION_FINISH 代表該筆通訊已被處理
    IMEXPORT void CALBY LClearQueue (int TgrConn);             //清除命令
    //  ============================= Loop read ============
    IMEXPORT void CALBY LReadBegin (int TgrConn);        //設定自動組合旗標
    IMEXPORT int  CALBY LReadNI (int TgrConn, int addr, int num);
    IMEXPORT int  CALBY LReadNO (int TgrConn, int addr, int num);
    IMEXPORT int  CALBY LReadNC (int TgrConn, int addr, int num);
    IMEXPORT int  CALBY LReadNS (int TgrConn, int addr, int num);
    IMEXPORT int  CALBY LReadNA (int TgrConn, int addr, int num);
    IMEXPORT int  CALBY LReadNR (int TgrConn, int addr, int num );
    IMEXPORT int  CALBY LReadNF (int TgrConn, int addr, int num );
    IMEXPORT int  CALBY LReadEnd(int TgrConn);        //完成自動組合設定並開始產生組合封包
    //-----
    /*
    IMEXPORT int  CALBY LReadNTimerSet (int TgrConn, int addr, int num );
    IMEXPORT int  CALBY LReadNTimerVal (int TgrConn, int addr, int num );
    IMEXPORT int  CALBY LReadNTimerTrg (int TgrConn, int addr, int num );
    IMEXPORT int  CALBY LReadNCounterSet (int TgrConn, int addr, int num );
    IMEXPORT int  CALBY LReadNCounterVal (int TgrConn, int addr, int num );
    IMEXPORT int  CALBY LReadNCounterTrg (int TgrConn, int addr, int num );
    */

    //  ============================= L write ============
    IMEXPORT void CALBY LWriteBegin (int TgrConn);        //設定自動組合旗標
    IMEXPORT int  CALBY LWriteNO (int TgrConn,  int addr, int num);
    IMEXPORT int  CALBY LWriteNC (int TgrConn,  int addr, int num);
    IMEXPORT int  CALBY LWriteNS (int TgrConn,  int addr, int num);
    IMEXPORT int  CALBY LWriteNA (int TgrConn,  int addr, int num);
    IMEXPORT int  CALBY LWriteNR (int TgrConn,  int addr, int num);
    IMEXPORT int  CALBY LWriteNF (int TgrConn,  int addr, int num);
    IMEXPORT int  CALBY LWriteEnd(int TgrConn);        //完成自動組合設定並開始產生組合封包


    IMEXPORT void CALBY DClearQueue (int TgrConn);             //清除命令
    IMEXPORT int  CALBY DWaitDone (int TgrConn, int MaxWaitTime);  //等待直接命令完成
    //  ============================= Direct read ============
    IMEXPORT void CALBY DReadBegin (int TgrConn);       //設定自動組合旗標
    IMEXPORT int  CALBY DReadNI (int TgrConn, int addr, int num);
    IMEXPORT int  CALBY DReadNO (int TgrConn, int addr, int num);
    IMEXPORT int  CALBY DReadNC (int TgrConn, int addr, int num);
    IMEXPORT int  CALBY DReadNS (int TgrConn, int addr, int num);
    IMEXPORT int  CALBY DReadNA (int TgrConn, int addr, int num);
    IMEXPORT int  CALBY DReadNR (int TgrConn, int addr, int num );
    IMEXPORT int  CALBY DReadNF (int TgrConn, int addr, int num );
    IMEXPORT int  CALBY DReadEnd (int TgrConn);      //完成自動組合設定並開始產生組合封包

    //  ============================= Direct write ============
    IMEXPORT void CALBY DWriteBegin (int TgrConn);       //設定自動組合旗標
    IMEXPORT int  CALBY DWrite1O (int TgrConn, int addr, int val);
    IMEXPORT int  CALBY DWrite1C (int TgrConn, int addr, int val);
    IMEXPORT int  CALBY DWrite1S (int TgrConn, int addr, int val);
    IMEXPORT int  CALBY DWrite1A (int TgrConn, int addr, int val);
    IMEXPORT int  CALBY DWrite1R (int TgrConn,  int addr, int val);
    IMEXPORT int  CALBY DWrite1F (int TgrConn,  int addr, double val);
    IMEXPORT int  CALBY DWrite1RBit (int TgrConn, int addr, int BitIdx, int BitValue);             //寫入 R bit
    IMEXPORT int  CALBY DWriteRString (int TgrConn, int addr, int RSize, char *Buf);            //寫入字串到 R
    IMEXPORT int  CALBY DWriteNO (int TgrConn,  int addr, int num, int *data);
    IMEXPORT int  CALBY DWriteNC (int TgrConn,  int addr, int num, int *data);
    IMEXPORT int  CALBY DWriteNS (int TgrConn,  int addr, int num, int *data);
    IMEXPORT int  CALBY DWriteNA (int TgrConn,  int addr, int num, int *data);
    IMEXPORT int  CALBY DWriteNR (int TgrConn,  int addr, int num, int *data);
    IMEXPORT int  CALBY DWriteNF (int TgrConn,  int addr, int num, double *data);
    IMEXPORT int  CALBY DWriteEnd (int TgrConn);      //完成自動組合設定並開始產生組合封包
};
#endif // SCIF2_H
