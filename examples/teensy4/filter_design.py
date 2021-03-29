#ntaps = 5
#decimation = 5
#interpolation = 3

ntaps = 50
decimation = 17
interpolation = 8

input_samples = list(range(1000))

interpolated_samples = []
for i in input_samples:
    for x in range(interpolation - 1):
        interpolated_samples.append(None)
    interpolated_samples.append(i)

cnt = 0
for output_sample in range(100):
    offset = output_sample * decimation
    parts = []
    max_i = 0
    min_i = None
    for (tap,i) in enumerate(interpolated_samples[offset:offset+ntaps]):
        if i is not None:
            parts.append(f"I[{i}] * F[{tap}]")
            max_i = tap
            if min_i is None:
                min_i = i
        pass
    s = " + ".join(parts)
    print(f"O[{output_sample}] = {s}")
    if output_sample > 0:
        if min_i % decimation == 0:
            print()
            cnt += 1
            if cnt == 2:
                break
    pass
