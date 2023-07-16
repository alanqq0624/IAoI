# IAoI
- Code of my paper "AirComp-aided Safety-aware CAM Broadcast Rate Control in C-V2X Sidelink"
- Simulator: OpenCV2X: [github](https://github.com/brianmc95/OpenCV2X), [website](https://www.cs.ucc.ie/~bm18/cv2x), [doc](./OpenCV2X_Documentation.pdf)
    - 2023/07/16: website gives error 403 ???
- server與github檔案位置對應
    - 162: `/home/mnetlab/SSD_2/alanqq0624/Veins/simulte_opencv2x_backup/`
    - github: `IAoI/`

## Version 
- omnet++ 5.6.2: 
    - 應該不要裝到6.x.x就好
- veins 5.2: [website](https://veins.car2x.org/)
- INET **3.6.6**: [website](https://inet.omnetpp.org/)
    - 這個版本一定要正確
- simulte: use OpenCV2X to replace it
- sumo 1.14.1: [website](https://eclipse.dev/sumo/)
    - sim. scenario: LuST scenario , [github](https://github.com/lcodeca/LuSTScenario)
        - 可能有與SUMO的版本衝突造成功能不正常，但是先這樣
        - 推薦改用github說明欄的MoST ?

## environment install
    雖然大概不需要，但是紀錄一下。
    大致上依循OpenCV2X的安裝說明: [doc](./OpenCV2X_Documentation.pdf)
1. 安裝omnet++
    - 照著omnet++的DOC裝
    - OpenCV2X的DOC有額外的調整說明
2. 下載對應版本的veins, INET(**3.6.6**)與openCV2X並找個解壓縮
3. 打開omnet++建立project並按照順序include
    1. `$ omnetpp`
    2. create workspace，但是不要從這邊引入INET
    3. 點選include位置: File | Import | General | Existing Projects into Workspace
    3. include順序
        1. INET (**3.6.6**)
        2. OpenCV2X
        3. Veins
            - 點 `Search for nested 
project` 來安裝 `veins` 與 `veins_inet3` 
4. 照著[OpenCV2X_DOC](./OpenCV2X_Documentation.pdf)中的1.1.4調整
5. 安裝SUMO
    - use `apt` ?

## Running
0. 確認omnetpp中的所有程式都有編譯成功
1. running veins_launchd
    - [command_doc](https://veins.car2x.org/documentation/sumo-launchd/)
    - `./veins_launchd -vv -t -c sumo`
    - sumo可換成sumo-gui
2. 在 lte | simulations | Mode4_CAM | omnetpp.ini點下run按鈕
    - 162: `/home/mnetlab/SSD_2/alanqq0624/Veins/simulte_opencv2x_backup/simulte/simulations/Mode4_CAM`
3. 執行中...
    - 前面可能有預跑sumo部分，請等待
4. 執行完成後，在介面按下確定後，請等待前面執行完成`veins_launchd`
    - 原本的veins不會有倒數
5. 記得去`/tmp`裡面把SUMO資料複製出來，以免被洗掉

---
## Simulation result

- 所需的python檔都放在
    - github: `IAoI/python_plot
/sumo_CR.ipynb`

### Network (omnetpp)
- Result at lte | simulations | Mode4_CAM | result
- sca, vec, vci分別儲存不同的statistic結果
- 可以直接點兩下開啟檢視介面快速檢視數據
- output: [OpenCV2X_DOC](./OpenCV2X_Documentation.pdf)中的第3部分
    1. 用`scavetool`把以上結果輸出成csv
    2. 用python讀取想要的數據並繪圖
        - github: `IAoI/python_plot
/sumo_period_PDF.ipynb`

### Safety (SUMO)
0. 確認執行的`veins_launchd`時有加`-t`
1. 找出執行完成時複製出來的sumo資料
    - 執行完的時候，就要去`/tmp`裡面把SUMO資料複製出來，以免被洗掉
2. 跑python `sumo_plot.ipynb`, `sumo_plot_2.ipynb`，可以從SSM與其他輸出檔分別抓出不同資料成csv
3. 跑其他的python檔來畫圖


---
## Important file location
- 基本上帶有 **mode4** 的檔案就是重要的
- `/home/mnetlab/SSD_2/alanqq0624/Veins/simulte_opencv2x_backup/simulte/src/apps/mode4App`: 
    載具所搭載程式，application layer 部分
    - `Mode4Aircomp`: My work
    - `Mode4CAM`: compare scheme of TAoI
    - `Mode4App`: base
    - `*_m.*`: 由`*.msg`編譯產生的
- `/home/mnetlab/SSD_2/alanqq0624/Veins/simulte_opencv2x_backup/simulte/src/stack/phy`
    - PHY layer
    - `packet`: 傳送的packet
    - `layer/LtePhyVUeMode4`: mode4相關PHY實作
- `/home/mnetlab/SSD_2/alanqq0624/Veins/simulte_opencv2x_backup/simulte/src/stack/phy`
    - MAC layer
    - `packet`: 傳送的packet
    - `layer/LteMacVUeMode4`: mode4相關MAC實作
    - `buffer`: 裡面有mode4相關HARQ實作
- `/home/mnetlab/SSD_2/alanqq0624/Veins/simulte_opencv2x_backup/simulte/simulation/Mode4_CAM`
    - simulation的設定與結果輸出
    - `omnetpp.ini`: simulation參數設定
    - `highway`: 放scenario
        - `LS_001`: 簡單的工字型車道
        - `lust_*`: LuST Scenario
        - **`*.sumocfg`: scenario的config設定**
    - `result`: 輸出結果位置之一


