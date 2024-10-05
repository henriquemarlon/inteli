use warp::Filter;

#[tokio::main]
async fn main() {
    let html_content = warp::path::end()
        .map(|| {
            warp::reply::html(include_str!("./index.html"))
        })
        .with(warp::cors().allow_any_origin());
    warp::serve(html_content)
        .run(([0, 0, 0, 0], 3000))
        .await;
}
