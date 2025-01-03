# Raytracer bonus

Реализуйте всю функциональность в основном `raytracer`,
на сервере будут учитываться файлы оттуда.

В этой задаче вам предстоит ускорить ваш рейтрейсер, чтобы с помощью него можно было рендерить более сложные
сцены. Большинство объектов обычно представлены тысячами или миллионами полигонов. Нетрудно понять,
что базовая реализация, которая производит линейный проход по всем полигонам с целью найти ближайший, не способна
отрендерить хоть сколько нибудь сложные объекты за приемлемое время.

Используйте алгоритм `Bounding volume hierarchy` (BVH) для ускорения этого поиска. В нашей задаче строить иерархию нужно только для треугольников.

* Презентацию с описанием можно найти здесь: http://15462.courses.cs.cmu.edu/fall2019content/lectures/15_spatialdatastructures/15_spatialdatastructures_slides.pdf. Там же приведен способ, которым можно построить достаточно хорошую иерархию.
* Описание на вики: https://en.wikipedia.org/wiki/Bounding_volume_hierarchy
* При реализации пересечения луча и bounding box не забудьте правильно обработать случай, когда начало луча
лежит внутри этого box.
* Тест `Figure` не запускается на сервере и не включен в репозиторий, т.к. он слишком большой. Вы можете скачать .obj файл для него здесь: https://drive.google.com/file/d/1Ygw9FlJkzx78lFOAykd4E6MiorAzE63C/view?usp=sharing
* Решение проверяется в `Asan` и `RelWithDebInfo` сборках.
В `Asan` сборке запускается только тест `Bunny`.
* Для сдачи задания поменяйте содержимое файла stub.txt
и отправте эти изменения в свой удалённый репозиторий.