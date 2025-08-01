
curl -X PUT "https://api.particle.io/v1/products/14950/firmware/release" \
               -H "Authorization: Bearer b2e94c349dadc106318e78e8e404b849758d2f74" \
               -d version=8 \
               -d groups[]="kuhmute&groups[]=test"\