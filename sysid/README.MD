```
FILE= ; mkdir logs_csv/$FILE; ulog2csv logs/$FILE -o logs_csv/$FILE
```


High-frequency logging, in the mavlink console:
```
logger stop
logger start -r 2000 -b 100
```