알고계시겠지만 저희는 현재 기업 과제를 하고 있습니다.
chip_data안에 있는 칩을 저희가 만든 시뮬레이터로 렌더링하면 chip_rendered 안의 그림과 같이 나옵니다.
여기서 differentiable rendering을 수행해서 알맞은 material parameter를 찾는것이 최종적으로 해야할 목표입니다.
일단은 저희가 렌더링한 이미지를 reference로 삼아서 하겠지만 실제 이미지에 대해서도 적용해 볼 수 있도록 해볼 예정입니다.
하셔야 할 일은 다음과 같습니다.

(1) Geometry & Material
저희가 사용한 렌더러는 mitsuba가 아니라서 format이 약간 다릅니다.
일단 geometry가 heightmap으로 주어져있고 각각의 material이 labelmap으로 주어져있습니다.
이를 mitsuba format에 맞게 obj 파일들로 만들어야합니다.
scale에 대한 정보는 general_info.ini에 ScaleFactorX/Y로 정의되어 있습니다.
chip은 정 중앙에 위치하고 있다고 생각해주세요.
각각의 label에 대한 material은 material_info.ini에 정의되어 있습니다.
역시 포맷이 달라서 mitsuba형식으로 정의해줘야 합니다.
아마 diffuse와 conductor, plastic정도면 충분할거 같네요.
정리하자면 heightmap과 label map을 읽어서 label(1~5)마다 geometry를 만든 뒤 
각각 obj file로 내보내고 scene.xml에 shape과 material을 추가하면 됩니다.

(2) Light
geometry와 material에 대한 준비가 끝났다면 다음은 light입니다.
light도 scene.xml에 추가 해줘야 합니다.
top, mid, bottom 3가지가 있는데 mid, bottom만 고려하면 됩니다.
mid, bottom은 diffuse(area) light로 12개의 작은 직사각형 조명이 링 형태로 배열되어 있습니다.(그림1 참고)
구체적인 수치는 그림 2를 참고하세요.
시뮬레이터 코드 일부도 같이 드렸으니 참고하시면 될거 같습니다.

(3) Camera
이제 마지막으로 camera의 위치를 설정해야 합니다.
camera에 대한 정보는 그림(3)에 있습니다.
orthographic camera를 사용했는데 공식적인 mitsuba2 release에는 orthographic camera가 없습니다.
github에 보시면 유저가 orthographic camera를 추가해놓은게 있으니 그걸 사용하면 될거 같습니다만
일단 (1), (2)를 하고 난 뒤에 천천히 생각해도 될거 같습니다.


(4)
(1), (2), (3)을 다 하셨으면 이제 scene.xml과 obj file들이 생길겁니다.
이제 이들을 이용해 mitsuba render를 돌리고 differentiable rendering을 수행하시면 됩니다.