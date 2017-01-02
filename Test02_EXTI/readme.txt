1,pinoutタブでpinにGPIO_EXTIxxに設定。
2,ConfigurationタブのGPIOメニューでさっき設定したピンのGPIOmodeを使いたい外部割り込の検出設定に合わせる。(立ち上がり/立ち下がり/両方)
3,ConfigurationタブのNVIC設定メニューでEXTI lineと書いてある項目を見つけ、Enabledにチェックをつける。
4,コードを生成。
5,/*USER CODE BEGIN 4*/と/*USER CODE END 4*/の間にHAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)の関数を宣言し、内容を記述する。この関数の内容が割り込んだ時に実行される処理になる。
6,make,make flashで書き込み、動作確認する。
