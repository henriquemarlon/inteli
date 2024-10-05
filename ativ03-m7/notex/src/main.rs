use bcrypt::{hash, verify};
use jsonwebtoken::errors::Error as JWTError;
use jsonwebtoken::{decode, encode, Algorithm, DecodingKey, EncodingKey, Header, Validation};
use postgres::Error as PostgresError;
use postgres::{Client, NoTls};
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};

#[macro_use]
extern crate serde_derive;

#[macro_use]
extern crate dotenv_codegen;

#[derive(Serialize, Deserialize)]
struct RequestData {
    header: String,
    body: String,
}

#[derive(Debug, Serialize, Deserialize)]
struct Claims {
    iss: String,
    sub: String,
    exp: usize,
}

//Model: User struct with id, name, email
#[derive(Serialize, Deserialize)]
struct User {
    id: Option<i32>,
    username: String,
    email: String,
    password: String,
}

//Model: Post struct with id, created_at, updated_at, title, content, author_id
#[derive(Serialize, Deserialize)]
struct Post {
    id: Option<i32>,
    title: String,
    content: String,
    author_id: Option<i32>,
}

//SECRET
const SECRET: &[u8] = dotenv!("JWT_SECRET_KEY").as_bytes();

//DATABASE_URL
const DB_URL: &str = dotenv!("DATABASE_URL");

//constants
const OK_RESPONSE: &str = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nAccess-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: POST, GET, OPTIONS\r\nAccess-Control-Allow-Headers: Content-Type, Authorization\r\nAccess-Control-Allow-Credentials: true\r\nAccess-Control-Expose-Headers: Custom-Header1, Custom-Header2\r\nAccess-Control-Max-Age: 86400\r\n\r\n";
const NOT_FOUND: &str = "HTTP/1.1 404 NOT FOUND\r\n\r\n";
const INTERNAL_SERVER_ERROR: &str = "HTTP/1.1 500 INTERNAL SERVER ERROR\r\n\r\n";
const UNAUTHORIZED_RESPONSE: &str = "HTTP/1.1 401 UNAUTHORIZED\r\n\r\n";

//main function
fn main() {
    if let Err(e) = set_database() {
        println!("Error: {}", e);
        return;
    }
    let listener = TcpListener::bind(format!("0.0.0.0:8080")).unwrap();
    println!("Server started at port http://0.0.0.0:8080");
    for stream in listener.incoming() {
        match stream {
            Ok(stream) => {
                handle_client(stream);
            }
            Err(e) => {
                println!("Error: {}", e);
            }
        }
    }
}

//handle_client function
fn handle_client(mut stream: TcpStream) {
    let mut buffer = [0; 1024];
    let mut request = String::new();

    match stream.read(&mut buffer) {
        Ok(size) => {
            request.push_str(String::from_utf8_lossy(&buffer[..size]).as_ref());

            let (status_line, content) = match &*request {
                r if r.starts_with("OPTIONS") => (OK_RESPONSE.to_string(), "".to_string()),
                r if r.starts_with("POST /user/create") => create_user(r),
                r if r.starts_with("POST /user/login") => login(r),
                r if r.starts_with("POST /post/create") => create_post(r),
                r if r.starts_with("GET /post/get") => get_all_posts_by_author(r),
                _ => (NOT_FOUND.to_string(), "404 Not Found".to_string()),
            };

            let response = format!("{}{}", status_line, content);

            stream.write_all(response.as_bytes()).unwrap();
        }
        Err(e) => {
            println!("Error: {}", e);
        }
    }
}

//CONTROLLERS
fn create_user(_request: &str) -> (String, String) {
    let request_data = get_header_and_body_request(_request);
    let body: Result<User, serde_json::Error> = serde_json::from_str(&request_data.body);
    match (body, Client::connect(DB_URL, NoTls)) {
        (Ok(user), Ok(mut client)) => {
            let hashed_password = store_password(&user.password).unwrap();
            client
                .execute(
                    "INSERT INTO users (name, email, password) VALUES ($1, $2, $3)",
                    &[&user.username, &user.email, &hashed_password],
                )
                .unwrap();

            (OK_RESPONSE.to_string(), "User created".to_string())
        }
        _ => (INTERNAL_SERVER_ERROR.to_string(), "Error".to_string()),
    }
}

fn login(_request: &str) -> (String, String) {
    let request_data = get_header_and_body_request(_request);
    let body: Result<User, serde_json::Error> = serde_json::from_str(&request_data.body);
    match (body, Client::connect(DB_URL, NoTls)) {
        (Ok(user), Ok(mut client)) => {
            let row = client
                .query_one("SELECT * FROM users WHERE email = $1", &[&user.email])
                .unwrap();

            let user_data = User {
                id: row.get(0),
                username: row.get(1),
                email: row.get(2),
                password: row.get(3),
            };

            let is_valid = verify_password(&user.password, &user_data.password).unwrap();

            if user_data.username != user.username {
                return (
                    INTERNAL_SERVER_ERROR.to_string(),
                    "Invalid username".to_string(),
                );
            } else if user_data.email != user.email {
                return (
                    INTERNAL_SERVER_ERROR.to_string(),
                    "Invalid email".to_string(),
                );
            }
            if is_valid {
                let token = generate_token(&user_data.id.unwrap().to_string()).unwrap();
                return (OK_RESPONSE.to_string(), token);
            } else {
                return (
                    INTERNAL_SERVER_ERROR.to_string(),
                    "Invalid password".to_string(),
                );
            }
        }
        (Err(e), _) => (
            INTERNAL_SERVER_ERROR.to_string(),
            format!("Error parsing user data: {}", e),
        ),
        (_, Err(e)) => (
            INTERNAL_SERVER_ERROR.to_string(),
            format!("Error connecting to database: {}", e),
        ),
    }
}

fn create_post(_request: &str) -> (String, String) {
    let request_data = get_header_and_body_request(_request);
    let token = extract_token_from_header(&request_data.header);
    let body: Result<Post, serde_json::Error> = serde_json::from_str(&request_data.body);
    let token_validation = validate_token(token);
    let db_connection = Client::connect(DB_URL, NoTls);

    match (token_validation, body, db_connection) {
        (Ok(claim), Ok(post), Ok(mut client)) => {
            let author_id: i32 = claim.sub.parse().unwrap_or(0);
            if let Err(e) = client.execute(
                "INSERT INTO posts (title, content, author_id) VALUES ($1, $2, $3)",
                &[&post.title, &post.content, &author_id],
            ) {
                println!("Error: {}", e);
                return (
                    INTERNAL_SERVER_ERROR.to_string(),
                    "Error inserting post".to_string(),
                );
            }
            (OK_RESPONSE.to_string(), "Post created".to_string())
        }
        (Err(_), _, _) => (
            UNAUTHORIZED_RESPONSE.to_string(),
            "Invalid token".to_string(),
        ),
        (_, Err(_), _) => (
            INTERNAL_SERVER_ERROR.to_string(),
            "Error parsing post data".to_string(),
        ),
        (_, _, Err(_)) => (
            INTERNAL_SERVER_ERROR.to_string(),
            "Error connecting to database".to_string(),
        ),
    }
}

fn get_all_posts_by_author(_request: &str) -> (String, String) {
    let request_data = get_header_and_body_request(_request);
    let token_data = validate_token(extract_token_from_header(&request_data.header));
    match token_data {
        Ok(claim) => match Client::connect(DB_URL, NoTls) {
            Ok(mut client) => {
                let author_id: i32 = claim
                    .sub
                    .parse()
                    .expect("Failed to convert claim.sub to i32");
                let mut post_list = vec![];
                for row in client
                    .query("SELECT * FROM posts WHERE author_id = $1", &[&author_id])
                    .unwrap()
                {
                    let post = Post {
                        id: row.get(0),
                        title: row.get(1),
                        content: row.get(2),
                        author_id: row.get(3),
                    };
                    post_list.push(post);
                }
                (
                    OK_RESPONSE.to_string(),
                    serde_json::to_string(&post_list).unwrap(),
                )
            }
            Err(_) => (
                INTERNAL_SERVER_ERROR.to_string(),
                "Error connecting to database".to_string(),
            ),
        },
        Err(_) => (
            UNAUTHORIZED_RESPONSE.to_string(),
            "Invalid token".to_string(),
        ),
    }
}

fn set_database() -> Result<(), PostgresError> {
    dotenv::dotenv().ok();
    let mut client = Client::connect(DB_URL, NoTls)?;

    client.batch_execute(
        "CREATE TABLE IF NOT EXISTS users (
            id SERIAL PRIMARY KEY,
            name VARCHAR NOT NULL,
            email VARCHAR NOT NULL,
            password VARCHAR NOT NULL
        )",
    )?;

    client.batch_execute(
        "CREATE TABLE IF NOT EXISTS posts (
            id SERIAL PRIMARY KEY,
            title VARCHAR NOT NULL,
            content VARCHAR NOT NULL,
            author_id INTEGER NOT NULL REFERENCES users(id)
        )",
    )?;
    Ok(())
}

fn generate_token(user_id: &str) -> Result<String, JWTError> {
    dotenv::dotenv().ok();
    let encoding_key: EncodingKey = EncodingKey::from_secret(SECRET.as_ref());
    let expiration = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs() as usize
        + 3600;
    let claims = Claims {
        iss: "notex".to_string(),
        sub: user_id.to_string(),
        exp: expiration,
    };
    encode(&Header::default(), &claims, &encoding_key)
}

fn validate_token(token: &str) -> Result<Claims, JWTError> {
    dotenv::dotenv().ok();
    let encoding_key: DecodingKey = DecodingKey::from_secret(SECRET.as_ref());
    let mut issuers = std::collections::HashSet::new();
    issuers.insert("notex".to_string());
    let mut validation = Validation::new(Algorithm::HS256);
    validation.iss = Some(issuers);
    validation.leeway = 60;
    validation.validate_exp = true;
    validation.validate_nbf = false;
    validation.aud = None;
    validation.sub = None;
    decode::<Claims>(token, &encoding_key, &validation).map(|data| data.claims)
}

fn store_password(plain: &str) -> Result<String, bcrypt::BcryptError> {
    hash(plain, 4)
}

fn verify_password(plain: &str, hashed: &str) -> Result<bool, bcrypt::BcryptError> {
    verify(plain, hashed)
}

fn get_header_and_body_request(request: &str) -> RequestData {
    let req: Vec<&str> = request.split("\r\n\r\n").collect();
    let header = req.get(0).unwrap_or(&"").to_string();
    let body = req.get(1).unwrap_or(&"").to_string();
    RequestData { header, body }
}

fn extract_token_from_header(header: &str) -> &str {
    for line in header.lines() {
        if line.starts_with("Authorization: Bearer ") {
            return line.split_whitespace().nth(2).unwrap_or("");
        }
    }
    ""
}
