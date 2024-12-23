# Отчёт об ускорении

* Замеры проводились с помощью команды time в терминале Linux

* Сравнивалось время исполнение кода из двух веток - develop (ДЗ-2) и текущей fluid-3 (ДЗ-3) в однопоточном режиме

### Сравнение

|Кол-во тиков| До | После | Во сколько раз ускорилось|
|-|-|-|-|
|100 | 0,665 s | 0,446 s|  1,49 |
| 1000 | 3,458 s | 2,205 s| 1,56 |
| 5000 | 11,765 s | 7,759 s| 1,56 |

### Что изменилось

* Быстрый поиск нужной пары из deltas (src/simulation/vector_field.hpp):
```
switch  (2 + dx + 2*dy)
{
case  0: return v[x][y][2];
case  1: return v[x][y][0];
case  2: assert(false);
case  3: return v[x][y][1];
case  4: return v[x][y][3];
default: assert(false);
}
```

* Отбрасывание мелких значений потока при поиске в глубину (src/simulation/simulation.hpp):
```
void Simulation::propagate_flow( ... ) 
{
    ...

     // look at every nearby particle
    for (auto [dx, dy] : deltas) 
    {

        VF vp = std::min(lim, cap - flow);

        if (vp < 0.005) // THIS LINE ENHACE PERFORMANCE
            continue;     
        ...
    }
    ...
} 
```
