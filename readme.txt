�˰��ð����� ����� ���� ��� ������ �ϰ� �ֽ��ϴ�.
chip_data�ȿ� �ִ� Ĩ�� ���� ���� �ùķ����ͷ� �������ϸ� chip_rendered ���� �׸��� ���� ���ɴϴ�.
���⼭ differentiable rendering�� �����ؼ� �˸��� material parameter�� ã�°��� ���������� �ؾ��� ��ǥ�Դϴ�.
�ϴ��� ���� �������� �̹����� reference�� ��Ƽ� �ϰ����� ���� �̹����� ���ؼ��� ������ �� �� �ֵ��� �غ� �����Դϴ�.
�ϼž� �� ���� ������ �����ϴ�.

(1) Geometry & Material
���� ����� �������� mitsuba�� �ƴ϶� format�� �ణ �ٸ��ϴ�.
�ϴ� geometry�� heightmap���� �־����ְ� ������ material�� labelmap���� �־����ֽ��ϴ�.
�̸� mitsuba format�� �°� obj ���ϵ�� �������մϴ�.
scale�� ���� ������ general_info.ini�� ScaleFactorX/Y�� ���ǵǾ� �ֽ��ϴ�.
chip�� �� �߾ӿ� ��ġ�ϰ� �ִٰ� �������ּ���.
������ label�� ���� material�� material_info.ini�� ���ǵǾ� �ֽ��ϴ�.
���� ������ �޶� mitsuba�������� ��������� �մϴ�.
�Ƹ� diffuse�� conductor, plastic������ ����Ұ� ���׿�.
�������ڸ� heightmap�� label map�� �о label(1~5)���� geometry�� ���� �� 
���� obj file�� �������� scene.xml�� shape�� material�� �߰��ϸ� �˴ϴ�.

(2) Light
geometry�� material�� ���� �غ� �����ٸ� ������ light�Դϴ�.
light�� scene.xml�� �߰� ����� �մϴ�.
top, mid, bottom 3������ �ִµ� mid, bottom�� ����ϸ� �˴ϴ�.
mid, bottom�� diffuse(area) light�� 12���� ���� ���簢�� ������ �� ���·� �迭�Ǿ� �ֽ��ϴ�.(�׸�1 ����)
��ü���� ��ġ�� �׸� 2�� �����ϼ���.
�ùķ����� �ڵ� �Ϻε� ���� ������� �����Ͻø� �ɰ� �����ϴ�.

(3) Camera
���� ���������� camera�� ��ġ�� �����ؾ� �մϴ�.
camera�� ���� ������ �׸�(3)�� �ֽ��ϴ�.
orthographic camera�� ����ߴµ� �������� mitsuba2 release���� orthographic camera�� �����ϴ�.
github�� ���ø� ������ orthographic camera�� �߰��س����� ������ �װ� ����ϸ� �ɰ� �����ϴٸ�
�ϴ� (1), (2)�� �ϰ� �� �ڿ� õõ�� �����ص� �ɰ� �����ϴ�.


(4)
(1), (2), (3)�� �� �ϼ����� ���� scene.xml�� obj file���� ����̴ϴ�.
���� �̵��� �̿��� mitsuba render�� ������ differentiable rendering�� �����Ͻø� �˴ϴ�.